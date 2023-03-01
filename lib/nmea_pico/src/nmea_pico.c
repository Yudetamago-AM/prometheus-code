#include "nmea_pico.h"

// RX interrupt handler

void get_gps (uint8_t ch, GPS_data_t * GPS_data) {
    static int c; 
    int i; 
    char which_protocol[6], is_it_pstm[20]; 
    
    switch(ch) {
        case '$':
        c = 0; 
        break; 

        case '\n':
        sentence[c] = ch; 
        sentence[c + 1] = '\0'; 
        //which protocol?
        for(i = 0; i < 5; i++) {
            which_protocol[i] = sentence[i + 1]; 
            which_protocol[5] = '\0'; 
        } 
        if(strcmp(which_protocol, "GPRMC") == 0) {
            rmc = true; 
            edit_sentence(sentence, GPS_data); 
        } else if(strcmp(which_protocol, "GPGGA") == 0) {
            gga = true; 
            edit_sentence(sentence, GPS_data); 
        } 
        break; 

        default:
        break; 
    } 
    sentence[c] = ch; 
    c++; 
} 

int edit_sentence (char sentence[], GPS_data_t * GPS_data) {
    int i, comma, end; 
    char str; 
    size_t length = strlen(sentence); 
    //definition for checksum
    static long int checksum_cal; 
    static char checksum_true[4] = "0x"; 
    static int c_t; 
    bool after_checksum = false; 

    for(i = 0; i < length; i++) {
        if(after_checksum == true) {
            checksum_true[c_t] = sentence[i]; 
            c_t++; 
        }
        str = sentence[i];
        switch(str) {
            case '$':
            comma = 0; 
            break; 

            case ',':
            comma++; 
            if(rmc == true && comma >= 1 && comma <= 10) {
                i = into_struct_rmc(sentence, i, comma, GPS_data); 
            } else if(gga == true && comma >= 7 && comma <= 8) {
                i = into_struct_gga(sentence, i, comma, GPS_data); 
            } 
            break; 

            case '*':
            //printf("(''->checksum)\n"); 
            //for checksum
            c_t = 2; 
            after_checksum = true;
            break; 

            case '\n':
            //for checksum
            after_checksum = false; 
            checksum_true[0] = '\0'; 
            checksum_true[1] = '\0';
            checksum_true[2] = '\0';
            checksum_true[3] = '\0';
            //definition of true checksum in int
            long int checksum_true_int = strtol(checksum_true, NULL, 0); 
            //checksum comparison
            //might be ignoring so be deleted
            checksum_cal = get_checksum(sentence, 0, 0);
            if(checksum_cal == checksum_true_int) {
                printf("checksum match. imformation reliable.\n");
            } else{
                printf("checksum does not match. imformation may not be reliable.\n");
            }
            ready = true; 
            break; 
        } 
    } 
} 

int into_struct_rmc (char sentence[], int i, int comma, GPS_data_t *GPS_data) {
    int j, times; 
    //definition for web coordinate
    int web_coor_data; 
    static int web_coor_count; 

    switch(comma) {
        case 1:
        //printf("->1time\n"); 
        times = 1000000; 
        GPS_data->time = 0; 
        for(j = i + 1; sentence[j] != ','; j++) {
            if(sentence[j] == '.') {
                continue; 
            } else {
                GPS_data->time += (sentence[j] - 48) * times; 
            } 
            times /= 10; 
        } 
        GPS_data->time_hour = 0; 
        GPS_data->time_min = 0; 
        GPS_data->time_sec = 0; 
        GPS_data->time_hour = GPS_data->time / 100000; 
        GPS_data->time_min = (GPS_data->time  - GPS_data->time_hour * 100000) / 1000; 
        GPS_data->time_sec = (GPS_data->time  - GPS_data->time_hour * 100000 - GPS_data->time_min * 1000) / 10; 
        j--; 
        break; 

        case 2:
        for(j = i + 1; sentence[j] != ','; j++) {
        } 
        j--; 
        break; 

        case 3:
        //printf("->3latitude\n"); 
        times = 1000000; 
        GPS_data->latitude = 0; 
        web_coor_count = 1; 
        web_coor_data = 0; 
        for(j = i + 1; sentence[j] != ','; j++) {
            if(sentence[j] == '.') {
                continue; 
            } else {
                if(web_coor_count >= 1 && web_coor_count <= 2) {
                    GPS_data->latitude += (sentence[j] - 48) * times; 
                } else {
                    web_coor_data += (sentence[j] - 48) * times; 
                } 
            } 
            web_coor_count++; 
            times /= 10; 
        } 
        GPS_data->latitude += web_coor_data / 60; 
        j--; 
        //printf("(*''* 3finished)\n"); 
        break; 

        case 4:
        //printf("->4North/South\n"); 
        for(j = i + 1; sentence[j] != ','; j++) {
            if(sentence[i + 1] == 'S') {
                GPS_data->latitude = - GPS_data->latitude; 
            } 
        } 
        j--; 
        //printf("(*''* 4finished)\n"); 
        break; 

        case 5:
        //printf("->5longitude\n"); 
        times = 1000000; 
        GPS_data->longitude = 0; 
        web_coor_count = 1; 
        web_coor_data = 0; 
        for(j = i + 1; sentence[j] != ','; j++) {
            if(sentence[j] == '.') {
                continue; 
            } else {
                if(web_coor_count >= 1 && web_coor_count <= 3) {
                    GPS_data->longitude += (sentence[j] - 48) * times; 
                } else {
                    web_coor_data += (sentence[j] - 48) * times; 
                } 
            } 
            web_coor_count++; 
            times /= 10; 
        } 
        GPS_data->longitude += web_coor_data / 60; 
        j--; 
        //printf("(*''* 5finished)\n"); 
        break; 

        case 6:
        //printf("->6East/West\n"); 
        for(j = i + 1; sentence[j] != ','; j++) {
            if(sentence[i + 1] == 'S') {
                GPS_data->longitude = - GPS_data->longitude; 
            } 
        } 
        j--; 
        //printf("(*''* 6finished)\n"); 
        break; 

        case 7:
        //printf("->7Speed\n"); 
        times = 10; 
        GPS_data->speed = 0; 
        for(j = i + 1; sentence[j] != ','; j++) {
            if(sentence[j] == '.') {
                continue; 
            } else {
                GPS_data->speed += (sentence[j] - 48) * times; 
            } 
            times /= 10; 
        } 
        GPS_data->speed += GPS_data->speed; 
        j--; 
        //printf("(*''* 7finished)\n"); 
        break; 

        case 8:
        for(j = i + 1; sentence[j] != ','; j++)
        break; 

        case 9:
        times = 100000; 
        GPS_data->date = 0; 
        for(j = i + 1; sentence[j] != ','; j++) {
            GPS_data->date += (sentence[j] - 48) * times; 
            times /= 10; 
        } 
        GPS_data->date_day = 0; 
        GPS_data->date_month = 0; 
        GPS_data->date_year = 0; 
        GPS_data->date_day = GPS_data->date / 10000; 
        GPS_data->date_month = (GPS_data->date  - GPS_data->date_day * 10000) / 100; 
        GPS_data->date_year = GPS_data->date  - GPS_data->date_day * 10000 - GPS_data->date_month * 100; 
        j--; 
        break; 

        case 10:
        rmc = false; 
        break;           

        default:
        break;      
    } 
    return j; 
} 

int into_struct_gga (char sentence[], int i, int comma, GPS_data_t *GPS_data) {
    int j, times; 
    int web_coor_data; 
    static int web_coor_count; 

    switch(comma) {
        case 2:
        //printf("->2latitude\n");
        times = 1000000; 
        GPS_data->latitude = 0; 
        web_coor_count = 1; 
        web_coor_data = 0; 
        for(j = i + 1; sentence[j] != ','; j++) {
            if(sentence[j] == '.') {
                //printf("found dot\n"); 
                continue; 
            } else {
                if(web_coor_count >= 1 && web_coor_count <= 2) {
                    GPS_data->latitude += (sentence[j] - 48) * times; 
                } else {
                    web_coor_data += (sentence[j] - 48) * times; 
                } 
            } 
            web_coor_count++; 
            times /= 10; 
        } 
        GPS_data->latitude += web_coor_data / 60; 
        j--; 
        //printf("(*''* 2finished)\n"); 
        break; 

        case 3:
        //printf("->3North/South\n"); 
        for(j = i + 1; sentence[j] != ','; j++) {
            if(sentence[i + 1] == 'S') {
                GPS_data->latitude = - GPS_data->latitude; 
            } 
        } 
        j--; 
        //printf("(*''* 3finished)\n"); 
        break; 

        case 4:
        //printf("->4longitude\n"); 
        times = 1000000; 
        GPS_data->longitude = 0; 
        web_coor_count = 1; 
        web_coor_data = 0; 
        for(j = i + 1; sentence[j] != ','; j++) {
            if(sentence[j] == '.') {
                continue; 
            } else {
                if(web_coor_count >= 1 && web_coor_count <= 3) {
                    GPS_data->longitude += (sentence[j] - 48) * times; 
                } else {
                    web_coor_data += (sentence[j] - 48) * times; 
                } 
            } 
            web_coor_count++; 
            times /= 10; 
        } 
        GPS_data->longitude += web_coor_data / 60; 
        j--; 
        //printf("(*''* 4finished)\n"); 
        break; 

        case 5:
        //printf("->5East/West\n"); 
        for(j = i + 1; sentence[j] != ','; j++) {
            if(sentence[i + 1] == 'S') {
                GPS_data->longitude = - GPS_data->longitude; 
            } 
        } 
        j--; 
        //printf("(*''* 5inished)\n"); 
        break; 

        case 6:
        for(j = i + 1; sentence[j] != ','; j++) {
        } 
        j--; 
        break; 

        case 7:
        times = 10; 
        GPS_data->satellites = 0; 
        for(j = i + 1; sentence[j] != ','; j++) {
            GPS_data->satellites += (sentence[j] - 48) * times; 
            times /= 10; 
        } 
        break; 

        case 8:
        gga = false; 
        break; 
    } 
    return j; 
} 

//use this to get checksum (eg. when sending commands)
int get_checksum (char sentence[], int start, int finish){
    size_t length = strlen(sentence); 
    int checksum_cal = 0; 
    int i;

    if(start == 0 || finish == 0){
        for(i = 0;i < 100;i++){
            if(sentence[i] == '$') continue;
            if(sentence[i] == '*') break;
            checksum_cal ^= sentence[i];
        }
    }else{
        for(i = start - 1;i <= finish - 1;i++){
            checksum_cal ^= sentence[i];
        }
    }

    return checksum_cal;
}