#include "hal/stm32/console.h"

#include <cstdio>

#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

#include "config/board_feather.h"
#include "hal/console.h"
#include "hal/stm32/clock.h"
#include "hal/stm32/console_rings.h"
#include "hal/stm32/gpio.h"
#include "hal/stm32/irq.h"

namespace
{
using hal::stm32::DmaRxRing;
using hal::stm32::TxRing;

// The register names and the interrupt handlers below are fixed at USART3 on
// DMA1 channel 4, streams 3 (TX) and 1 (RX); fail the build if the board
// table asks for anything else.
static_assert(board::CONSOLE.usart == 3 && board::CONSOLE.dma == 1 &&
                  board::CONSOLE.dma_channel == 4 && board::CONSOLE.tx_stream == 3 &&
                  board::CONSOLE.rx_stream == 1,
              "console glue is written for USART3 on DMA1 ch4 s3/s1");
constexpr uint32_t USART     = USART3;
constexpr uint32_t DMA       = DMA1;
constexpr uint8_t  TX_STREAM = 3;
constexpr uint8_t  RX_STREAM = 1;

// USART3 is on APB1; 42 MHz / 1 Mbaud = 42 exactly, so no baud error.
constexpr hal::stm32::UsartBrr BRR =
    hal::stm32::usartBrr(hal::stm32::BUSES.pclk1_hz, board::CONSOLE.baud);
static_assert(BRR.exact, "console baud rate does not divide the APB1 clock");

// In .bss, which the linker script keeps in SRAM: the DMA cannot reach CCM.
TxRing<2048>      tx_ring;
DmaRxRing<256>    rx_ring;
volatile uint32_t tx_in_flight   = 0; // bytes the TX stream is sending; 0 when idle
volatile uint32_t rx_idle_events = 0;
volatile uint32_t rx_overruns    = 0;

char stdout_buffer[128];

// Starts the next DMA transfer if the stream is idle. Runs in the DMA
// interrupt, or in the main loop with interrupts masked.
void kickTx()
{
    if (tx_in_flight != 0)
        return;
    const auto span = tx_ring.nextSpan();
    if (span.len == 0)
        return;
    tx_in_flight = span.len;
    dma_clear_interrupt_flags(DMA, TX_STREAM,
                              DMA_TCIF | DMA_HTIF | DMA_TEIF | DMA_DMEIF | DMA_FEIF);
    dma_set_memory_address(DMA, TX_STREAM, reinterpret_cast<uint32_t>(span.data));
    dma_set_number_of_data(DMA, TX_STREAM, static_cast<uint16_t>(span.len));
    dma_enable_stream(DMA, TX_STREAM);
}

void setupTxStream()
{
    dma_stream_reset(DMA, TX_STREAM);
    dma_channel_select(DMA, TX_STREAM, DMA_SxCR_CHSEL_4);
    dma_set_transfer_mode(DMA, TX_STREAM, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
    dma_set_peripheral_address(DMA, TX_STREAM, reinterpret_cast<uint32_t>(&USART_DR(USART)));
    dma_set_memory_size(DMA, TX_STREAM, DMA_SxCR_MSIZE_8BIT);
    dma_set_peripheral_size(DMA, TX_STREAM, DMA_SxCR_PSIZE_8BIT);
    dma_enable_memory_increment_mode(DMA, TX_STREAM);
    dma_set_priority(DMA, TX_STREAM, DMA_SxCR_PL_LOW);
    dma_enable_transfer_complete_interrupt(DMA, TX_STREAM);
    dma_enable_transfer_error_interrupt(DMA, TX_STREAM);
    nvic_set_priority(NVIC_DMA1_STREAM3_IRQ, hal::stm32::irq::CONSOLE);
    nvic_enable_irq(NVIC_DMA1_STREAM3_IRQ);
}

void setupRxStream()
{
    dma_stream_reset(DMA, RX_STREAM);
    dma_channel_select(DMA, RX_STREAM, DMA_SxCR_CHSEL_4);
    dma_set_transfer_mode(DMA, RX_STREAM, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_set_peripheral_address(DMA, RX_STREAM, reinterpret_cast<uint32_t>(&USART_DR(USART)));
    dma_set_memory_address(DMA, RX_STREAM, reinterpret_cast<uint32_t>(rx_ring.buffer()));
    dma_set_number_of_data(DMA, RX_STREAM, rx_ring.SIZE);
    dma_set_memory_size(DMA, RX_STREAM, DMA_SxCR_MSIZE_8BIT);
    dma_set_peripheral_size(DMA, RX_STREAM, DMA_SxCR_PSIZE_8BIT);
    dma_enable_memory_increment_mode(DMA, RX_STREAM);
    dma_enable_circular_mode(DMA, RX_STREAM);
    dma_set_priority(DMA, RX_STREAM, DMA_SxCR_PL_HIGH);
    dma_enable_stream(DMA, RX_STREAM);
}
} // namespace

namespace hal::console
{
void stm32::init()
{
    hal::stm32::gpioAlternate(board::CONSOLE.tx.pin, board::CONSOLE.tx.af);
    // Pull RX up so an unplugged adapter reads as an idle line, not noise.
    hal::stm32::gpioAlternate(board::CONSOLE.rx.pin, board::CONSOLE.rx.af, GPIO_PUPD_PULLUP);

    rcc_periph_clock_enable(RCC_DMA1);
    rcc_periph_clock_enable(RCC_USART3);
    rcc_periph_reset_pulse(RST_USART3);

    setupRxStream();
    setupTxStream();

    // BRR written directly: usart_set_baudrate() rounds from rcc_apb1_frequency
    // at run time, while BRR here is checked exact at compile time.
    USART_BRR(USART) = BRR.brr;
    // EIE: with DMA receive on, an overrun (and framing or noise error) raises
    // the USART interrupt, so rxOverruns() sees every one, not only those
    // that land with an idle line.
    USART_CR3(USART) = USART_CR3_DMAT | USART_CR3_DMAR | USART_CR3_EIE;
    USART_CR1(USART) = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE;
    nvic_set_priority(NVIC_USART3_IRQ, hal::stm32::irq::CONSOLE);
    nvic_enable_irq(NVIC_USART3_IRQ);

    // Line-buffer printf into a static buffer, so newlib never mallocs one
    // and each line reaches _write in one piece.
    std::setvbuf(stdout, stdout_buffer, _IOLBF, sizeof stdout_buffer);
}

uint32_t stm32::txRingDroppedBytes()
{
    return tx_ring.dropped();
}

void stm32::resetTxRingDrops()
{
    tx_ring.resetDropped();
}

uint32_t stm32::rxIdleEvents()
{
    return rx_idle_events;
}

uint32_t stm32::rxOverruns()
{
    return rx_overruns;
}

int getcNonBlocking()
{
    return rx_ring.getc(DMA_SNDTR(DMA, RX_STREAM));
}

void detail::writeBytes(const char* buf, std::size_t len)
{
    tx_ring.push(buf, static_cast<uint32_t>(len));
    const uint32_t mask = cm_mask_interrupts(1);
    kickTx();
    cm_mask_interrupts(mask);
}
} // namespace hal::console

extern "C" void dma1_stream3_isr()
{
    // A transfer error also ends the transfer (the stream disables itself);
    // release its bytes either way so the console cannot jam.
    if (dma_get_interrupt_flag(DMA, TX_STREAM, DMA_TCIF | DMA_TEIF))
    {
        dma_clear_interrupt_flags(DMA, TX_STREAM, DMA_TCIF | DMA_TEIF);
        tx_ring.consume(tx_in_flight);
        tx_in_flight = 0;
        kickTx();
    }
}

extern "C" void usart3_isr()
{
    const uint32_t sr = USART_SR(USART);
    if (sr & (USART_SR_IDLE | USART_SR_ORE | USART_SR_FE | USART_SR_NE))
    {
        // RM0090: IDLE, ORE, FE and NE clear on a read of SR followed by a read of DR.
        // A byte that arrives between the two reads goes to this read, not
        // the DMA, and is lost; the window is a few cycles and the sequence
        // has no race-free alternative on this USART.
        (void)USART_DR(USART);
        if (sr & USART_SR_IDLE)
            rx_idle_events = rx_idle_events + 1;
        if (sr & USART_SR_ORE)
            rx_overruns = rx_overruns + 1;
    }
}

// newlib's write hook: printf and fwrite to stdout or stderr land here.
// Reports every byte as written, sent or dropped, so stdio never retries or blocks.
extern "C" int _write(int fd, const char* buf, int len)
{
    if (fd != 1 && fd != 2)
        return -1;
    if (len > 0)
        hal::console::write(buf, static_cast<std::size_t>(len));
    return len;
}
