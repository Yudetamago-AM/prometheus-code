#include "MAX-M10S_pico.h"

// RX interrupt handler
void on_uart_rx() {
    while (uart_is_readable(UART_ID)) {
        uint8_t ch = uart_getc(UART_ID);

        //rawdata
        //printf("%c", ch);

        //make sentence with ch
        make_sentence(ch);

        chars_rxed++;
    }
}

void make_sentence (uint8_t ch){
    static int c;
    int i;
    char which_protocol[6], is_it_pstm[20];
    
    switch(ch){
        case '$':
        c = 0;
        break;

        case '\n':
        sentence[c] = ch;
        sentence[c + 1] = '\0';
        //which protocol?
        for(i = 0;i < 5;i ++){
            which_protocol[i] = sentence[i + 1];
            which_protocol[5] = '\0';
        }
        if(strcmp(which_protocol, "GNRMC") == 0){
            rmc = true;
            edit_sentence(sentence);
        }else if(strcmp(which_protocol, "GNGGA") == 0){
            gga = true;
            edit_sentence(sentence);
        }
        break;

        default:
        break;
    }
    sentence[c] = ch;
    c++;
}

int edit_sentence (char sentence[]){
    int i, comma, end;
    char str;
    size_t length = strlen(sentence);
    //definition for checksum
    static long int checksum_cal;
    static char checksum_true[4] = "0x";
    static int c_t;
    bool checksum_ready = false;
    bool after_checksum = false;
    
    for(i = 0;i < length;i ++){
        if(checksum_ready == true && sentence[i] != '*'){
            checksum_cal ^= sentence[i];
        }
        if(after_checksum == true){
            checksum_true[c_t] = sentence[i];
            c_t++;
        }
        str = sentence[i];

        switch(str){
            case '$':
            comma = 0;
            //for checksum
            checksum_cal = 0;
            checksum_ready = true;
            break;

            case ',':
            comma ++;
            if(rmc == true && comma >= 1 && comma <= 10){
                i = into_struct_rmc(sentence, i, comma);
            }else if(gga == true && comma >= 7 && comma <= 8){
                i = into_struct_gga(sentence, i, comma);
            }
            break;

            case '*':
            //printf("(''->checksum)\n");
            //for checksum
            checksum_ready = false;
            c_t = 2;
            after_checksum = true;
            break;

            case '\n':
            //for checksum
            after_checksum = false;
            checksum_true[4] = '\0';
            //definition of true checksum in int
            long int checksum_true_int = strtol(checksum_true, NULL, 0);
            //checksum comparison
            if(checksum_cal == checksum_true_int){
            checksum_comparison = true;
            }
            ready = true;
            break;
        }
    }

}

int into_struct_rmc (char sentence[], int i, int comma){
    int j, times;
    //definition for web coordinate
    int web_coor_data;
    static int web_coor_count;

    switch(comma){
        case 1:
        //printf("->1time\n");
        times = 100000000;
        GPS_data.time = 0;
        for(j = i + 1;sentence[j] != ',';j ++){
            if(sentence[j] == '.') {
                continue;
            }else{
                GPS_data.time += (sentence[j] - 48) * times;
            }
            times /= 10;
        }
        GPS_data.time_hour = 0;
        GPS_data.time_min = 0;
        GPS_data.time_sec = 0;
        GPS_data.time_hour = GPS_data.time / 10000000;
        GPS_data.time_min = (GPS_data.time  - GPS_data.time_hour * 10000000) / 100000;
        GPS_data.time_sec = (GPS_data.time  - GPS_data.time_hour * 10000000 - GPS_data.time_min * 100000) / 1000;
        j --;
        break;

        case 2:
        for(j = i + 1;sentence[j] != ',';j ++){
        }
        j --;
        break;

        case 3:
        //printf("->3latitude\n");
        times = 100000000;
        GPS_data.latitude = 0;
        web_coor_count = 1;
        web_coor_data = 0;
        for(j = i + 1;sentence[j] != ',';j ++){
            if(sentence[j] == '.') {
                continue;
            }else{
                if(web_coor_count >= 1 && web_coor_count <= 2){
                    GPS_data.latitude += (sentence[j] - 48) * times;
                }else{
                    web_coor_data += (sentence[j] - 48) * times;
                }
            }
            web_coor_count ++;
            times /= 10;
        }
        GPS_data.latitude += web_coor_data / 60;
        j --;
        //printf("(*''* 3finished)\n");
        break;

        case 4:
        //printf("->4North/South\n");
        for(j = i + 1;sentence[j] != ',';j ++){
            if(sentence[i + 1] == 'S'){
                GPS_data.latitude = - GPS_data.latitude;
            }
        }
        j --;
        //printf("(*''* 4finished)\n");
        break;

        case 5:
        //printf("->5longitude\n");
        times = 1000000000;
        GPS_data.longitude = 0;
        web_coor_count = 1;
        web_coor_data = 0;
        for(j = i + 1;sentence[j] != ',';j ++){
            if(sentence[j] == '.') {
                continue;
            }else{
                if(web_coor_count >= 1 && web_coor_count <= 3){
                    GPS_data.longitude += (sentence[j] - 48) * times;
                }else{
                    web_coor_data += (sentence[j] - 48) * times;
                }
            }
            web_coor_count ++;
            times /= 10;
        }
        GPS_data.longitude += web_coor_data / 60;
        j --;
        //printf("(*''* 5finished)\n");
        break;

        case 6:
        //printf("->6East/West\n");
        for(j = i + 1;sentence[j] != ',';j ++){
            if(sentence[i + 1] == 'S'){
                GPS_data.longitude = - GPS_data.longitude;
            }
        }
        j --;
        //printf("(*''* 6finished)\n");
        break;

        case 7:
        //printf("->7Speed\n");
        times = 10;
        GPS_data.speed = 0;
        for(j = i + 1;sentence[j] != ',';j ++){
            if(sentence[j] == '.') {
                continue;
            }else{
                GPS_data.speed += (sentence[j] - 48) * times;
            }
            times /= 10;
        }
        GPS_data.speed += GPS_data.speed;
        j --;
        //printf("(*''* 7finished)\n");
        break;

        case 8:
        for(j = i + 1;sentence[j] != ',';j ++)
        break;

        case 9:
        times = 100000;
        GPS_data.date = 0;
        for(j = i + 1;sentence[j] != ',';j ++){
            GPS_data.date += (sentence[j] - 48) * times;
            times /= 10;
        }
        GPS_data.date_day = 0;
        GPS_data.date_month = 0;
        GPS_data.date_year = 0;
        GPS_data.date_day = GPS_data.date / 10000;
        GPS_data.date_month = (GPS_data.date  - GPS_data.date_day * 10000) / 100;
        GPS_data.date_year = (GPS_data.date  - GPS_data.date_day * 10000 - GPS_data.date_month * 100);
        j --;
        break;

        case 10:
        rmc = false;
        rmc_ready = true;
        break;          

        default:
        break;     
    }
    return j;
}

int into_struct_gga (char sentence[], int i, int comma){
    int j, times;
    int web_coor_data;
    static int web_coor_count;

    switch(comma){
        case 2:
        //printf("->2latitude\n");
        times = 100000000;
        GPS_data.latitude = 0;
        web_coor_count = 1;
        web_coor_data = 0;
        for(j = i + 1;sentence[j] != ',';j ++){
            if(sentence[j] == '.') {
                //printf("found dot\n");
                continue;
            }else{
                if(web_coor_count >= 1 && web_coor_count <= 2){
                    GPS_data.latitude += (sentence[j] - 48) * times;
                }else{
                    web_coor_data += (sentence[j] - 48) * times;
                }
                //printf("GPS_data.latitude:%d\n", GPS_data.latitude);
            }
            web_coor_count ++;
            times /= 10;
        }
        GPS_data.latitude += web_coor_data / 60;
        j --;
        //printf("(*''* 2finished)\n");
        break;

        case 3:
        //printf("->3North/South\n");
        for(j = i + 1;sentence[j] != ',';j ++){
            if(sentence[i + 1] == 'S'){
                GPS_data.latitude = - GPS_data.latitude;
            }
        }
        j --;
        //printf("(*''* 3finished)\n");
        break;

        case 4:
        //printf("->4longitude\n");
        times = 1000000000;
        GPS_data.longitude = 0;
        web_coor_count = 1;
        web_coor_data = 0;
        for(j = i + 1;sentence[j] != ',';j ++){
            if(sentence[j] == '.') {
                continue;
            }else{
                if(web_coor_count >= 1 && web_coor_count <= 3){
                    GPS_data.longitude += (sentence[j] - 48) * times;
                }else{
                    web_coor_data += (sentence[j] - 48) * times;
                }
            }
            web_coor_count ++;
            times /= 10;
        }
        GPS_data.longitude += web_coor_data / 60;
        j --;
        //printf("(*''* 4finished)\n");
        break;

        case 5:
        //printf("->5East/West\n");
        for(j = i + 1;sentence[j] != ',';j ++){
            if(sentence[i + 1] == 'S'){
                GPS_data.longitude = - GPS_data.longitude;
            }
        }
        j --;
        //printf("(*''* 5inished)\n");
        break;

        case 6:
        for(j = i + 1;sentence[j] != ',';j ++){
        }
        j --;
        break;

        case 7:
        times = 10;
        GPS_data.satellites = 0;
        for(j = i + 1;sentence[j] != ',';j ++){
            GPS_data.date += (sentence[j] - 48) * times;
            times /= 10;
        }
        break;

        case 8:
        gga = false;
        gga_ready = true;
        break;
    }
    return j;
}

int main() {

    //setup start
    stdio_init_all();

    // Set up our UART with a basic baud rate.
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Actually, we want a different speed
    // The call will return the actual baud rate selected, which will be as close as
    // possible to that requested
    int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);

    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, UART_PARITY_NONE);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);

    // OK, all set up.
    // Lets send a basic string out, and then run a loop and wait for RX interrupts
    // The handler will count them, but also reflect the incoming data back with a slight change!

   char buf[30] = "TEST";
   uart_puts(UART_ID, buf);

    while (1){
        tight_loop_contents();

        //output
        if(ready && rmc_ready){
            if(checksum_comparison = true){
                printf("checksum match. information reliable.\n");
            }else{
                printf("checksum no match. information not reliable.\n");
            }
            printf("('' -> rmc data begins)\n");
            printf("date:%d/%d/%d\n", GPS_data.date_year, GPS_data.date_month, GPS_data.date_day);
            printf("time:%d:%d:%d\n", GPS_data.time_hour, GPS_data.time_min, GPS_data.time_sec);
            printf("latitude:%d\n", GPS_data.latitude / 10000000);
            printf("longitude:%d\n", GPS_data.longitude / 10000000);
            printf("speed(knot):%d\n", GPS_data.speed);
            printf("('' ->   data_ends)\n");
            printf("\n");
            rmc_ready = false;
        }
        if(ready && gga_ready){
            if(checksum_comparison = true){
                printf("checksum match. information reliable.\n");
            }else{
                printf("checksum no match. information not reliable.\n");
            }
            printf("('' -> gga data begins)\n");
            printf("latitude:%d\n", GPS_data.latitude / 10000000);
            printf("longitude:%d\n", GPS_data.longitude / 10000000);
            printf("satellites:%d\n", GPS_data.satellites);
            printf("('' ->   data_ends)\n");
            printf("\n");
            gga_ready = false;
        }
        checksum_comparison = false;

        sleep_ms(1000);
    }

    //setup done

    return 0;
}

/// \end:uart_advanced[]