#include <stdio.h>
#include <string.h>

// Define buffer size
#define BUFFER_SIZE 256

// Buffers and indices
static char tx_buffer[BUFFER_SIZE]; // Transmit buffer
static char rx_buffer[BUFFER_SIZE]; // Receive buffer
static int tx_head = 0, tx_tail = 0; // Indices for transmit buffer
static int rx_head = 0, rx_tail = 0; // Indices for receive buffer

// Initialize UART (mock)
void UART_Init(unsigned long baud_rate) {
    printf("UART Initialized with baud rate %lu\n", baud_rate);
}

// Send a single byte of data via UART
void UART_SendByte(char data) {
    // Simulate adding data to the transmit buffer
    tx_buffer[tx_head] = data;
    tx_head = (tx_head + 1) % BUFFER_SIZE;
    printf("Sending byte: %c\n", data);
}

// Receive a single byte of data via UART
char UART_ReceiveByte(void) {
    if (rx_head == rx_tail) {
        // No data available to receive
        return '\0'; // Return null character if no data
    }

    // Simulate receiving data from the receive buffer
    char data_to_return = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % BUFFER_SIZE;
    printf("Receiving byte: %c\n", data_to_return);
    return data_to_return;
}

// Simulate UART data transfer (for testing)
void SimulateUARTTransfer(void) {
    // Example data to add to the receive buffer
    const char example_data[] = "Hello";

    // Simulate receiving data
    for (size_t i = 0; i < strlen(example_data); i++) {
        rx_buffer[rx_head] = example_data[i];
        rx_head = (rx_head + 1) % BUFFER_SIZE;
    }
}

int main(void) {
    UART_Init(9600); // Initialize UART

    // Simulate UART data transfer
    SimulateUARTTransfer();

    // Send data (simulating what would be sent over UART)
    const char data_to_send[] = "Hello";
    for (size_t i = 0; i < strlen(data_to_send); i++) {
        UART_SendByte(data_to_send[i]);
    }

    // Process received data and echo it back
    for (int i = 0; i < strlen(data_to_send); i++) { // Process length of data_to_send
        char received_char = UART_ReceiveByte(); // Receive a character
        if (received_char != '\0') { // Check if there is data
            UART_SendByte(received_char); // Echo the received character
        }
    }

    return 0;
}

