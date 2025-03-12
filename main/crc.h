#include <inttypes.h>

uint8_t duty_data[]={170,5,3,0,0,0,0,0,0,221};  /// =0 
uint8_t amper_data[]={170,5,4,0,0,0,0,0,0,221};  /// =0 
uint8_t speed_data[]={0xAA,0x05,0x27,0x00,0x00,0x00,0x00,0x07,0x10,0xDD};
/// crc для токового управления от 0 до 20ампер
uint16_t amper_crc[] = {
    0xC0D5,
    0x7ED5,
    0x6CD6,
    0x82d2,
    0x48d0,
    0x96d8,
    0xd4db,
    0xcade,
    0x00dc,
    0xeecc,
    0xfccf,
    0x22cb,
    0xe8c9,
    0x36c1,
    0xd4c3,
    0xcac6,
    0x00c4, 
    0x8ee4,
    0x9ce7,
    0x72e3,
    0xb8e1
};


volatile int erpm_step[] = {
    0,      // i = 0
    373,    // i = 1
    746,    // i = 2
    1119,   // i = 3
    1492,   // i = 4
    1865,   // i = 5
    2238,   // i = 6
    2611,   // i = 7
    2984,   // i = 8
    3358,   // i = 9
    3731,   // i = 10
    4104,   // i = 11
    4477,   // i = 12
    4700,   //     13 
    4850,   // i = 14
    5223,   // i = 15
    5596,   // i = 16
    5969,   // i = 17
    6342,   // i = 18
    6715,   // i = 19
    7088,   // i = 20
    7330,   //     21 
    7461,   // i = 22
    7834,   // i = 23
    8207,   // i = 24
    8580,   // i = 25
    8953,   // i = 26
    9326,   // i = 27
    9699,   // i = 28
    10073,  // i = 29
    10115,  // i = 30
    10446,  // i = 31    
    10819,  // i = 32
    11192,  // i = 33
    11565,  // i = 34
    11938,  // i = 35
    12311,  // i = 36
    12684,  // i = 37
    12870,   //38
    13057,  // i = 39
    13430,  // i = 40
    13803,  // i = 41
    14176,  // i = 42
    14549,  // i = 43
    14922,  // i = 44
    15295,  // i = 45
    15668,  // i = 46
    16041,  // i = 47
    16415,  // i = 48
    16788,  // i = 49
    17161,  // i = 50
    17534,  // i = 51
    17907,  // i = 52
    18280,   // i = 53
    20800   // 54
};

uint16_t erpm_crc[] = {
    0x0710,  //0
    0x70d0,
    0xe890,
    0xff52,
    0x0813,
    0xc1d3,
    0xb797,
    0xb256,
    0x8916,
    0x9f94,
    0x0a54,   //10
    0x011c,
    0x76dc,
    0x9e1c,
    0x229d,
    0xed5e,   //15
    0x0e1f,
    0x0bde,
    0x559a,
    0xb45a,
    0x431b,   //20
    0x7e99,   //21
    0x8cd8,   //22
    0xcc99,   //23
    0xc349,   //24
    0x3408,   //25 
    0xe5c8,
    0xeb8a,
    0x1e4b,
    0xcdcb,    //29
    0x564a,    //30
    0x938f,    //31
    0x964e,    //32
    0x850e,    //33
    0x4acd,    //34
    0x1e8c,     //35 
    0x0944,     //36
    0x3204,    //37
    0x9584,       //38
    0x37c5,    //39
    0x2187,    //40
    0x1847,    //41
    0x07ae,   //42
    0x58c2,   //43
    0x5083,  //44
    0x8742, //45
    0x4001, //46
    0x19c0, //47
    0x0f60, //48
    0xf821, //49
    0x31e1, //50
    0x27a3,  //51
    0xd262,  //52
    0x1922,  //53
    0x672c  //54
};


uint16_t crc16arc_bit(uint8_t *data, size_t len) {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (unsigned k = 0; k < 8; k++) {
            crc = crc & 1 ? (crc >> 1) ^ 0xA001 : crc >> 1;
        }
    }

    // Обработка второго байта
    uint8_t *bytePtr = (uint8_t *)&crc;
    uint8_t lastByte = bytePtr[0];
    int result = lastByte + 96;
    if (result > 127) {
        if (result & (1 << 7) && result & (1 << 5)) {
            result &= ~(1 << 7); // Если установлены 7 и 5 биты, сбрасываем 7
        } else if (result & (1 << 7) && result & (1 << 6)) {
            result &= ~(1 << 7); // Если установлены 7 и 6 биты, сбрасываем оба
            result &= ~(1 << 6);
        } else {
            result &= ~(1 << 7);
            result |= (1 << 6);
        }
    }
    bytePtr[0] = (uint8_t)result;
    return crc;
}


void convertToBytes(uint32_t number, uint8_t* bytes) {
    bytes[0] = (uint8_t)(number >> 24); // Старший байт
    bytes[1] = (uint8_t)(number >> 16);
    bytes[2] = (uint8_t)(number >> 8);
    bytes[3] = (uint8_t)number; // Младший байт
}



void getAmper(int number) {
    uint8_t bytes[4];
    uint16_t crc;
    convertToBytes(number * 1000, bytes);    
    for (int k = 0; k < sizeof(bytes); k++) {
        amper_data[k + 3] = bytes[k];
    }
    crc=amper_crc[number];
    amper_data[7] = (uint8_t)(crc >> 8); // Старший байт CRC
    amper_data[8] = (uint8_t)crc; // Младший байт CRC
}    


// Линейный поиск
int find_index_linear(int target) {
    for (int i = 0; i < sizeof(erpm_step); i++) {
        if (erpm_step[i] == target) {
            return i; // Точное совпадение
        }
    }
    return -1; // Не найдено
}

int getStep(bool forward,int value){
    int index = find_index_linear(value);  /// ищем текущий индех
    if (index<0)return 0;
    if (forward){ //вперед
        index++;
        if (index==sizeof(erpm_step))index--;            
    }
    else{
        index--;
        if (index<0)index=0;
    }
    return erpm_step[index];
 }

void getSpeed(int number) {
    //uint8_t bytes[4];
    uint16_t crc;    
    int index = find_index_linear(number);
    if (index != -1) {
    //convertToBytes(number * 1000, bytes);    
    speed_data[5] =(uint8_t)(number >> 8)& 0xFF; // Старший байт CRC
    speed_data[6] =(uint8_t) number& 0xFF; // Младший байт CRC
    crc=erpm_crc[index];
    speed_data[7] = (uint8_t)(crc >> 8); // Старший байт CRC
    speed_data[8] = (uint8_t)crc; // Младший байт CRC
    // for (int k = 0; k < sizeof(bytes); k++) {
    //     speed_data[k + 3] = bytes[k];
    // }
    }
    else {
        speed_data[5]=0;
        speed_data[6]=0;
        speed_data[7]=0x07;
        speed_data[8]=0x10;
    }
}  


void getDuty(float number) {
    uint8_t bytes[4];
    uint16_t crc;
    convertToBytes(number * 1000, bytes);
    
    for (int k = 0; k < sizeof(bytes); k++) {
        duty_data[k + 3] = bytes[k];
    }
    crc = crc16arc_bit(bytes, sizeof(bytes) / sizeof(bytes[0]));
    duty_data[7] = (uint8_t)(crc >> 8); // Старший байт CRC
    duty_data[8] = (uint8_t)crc; // Младший байт CRC
} 
