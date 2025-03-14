// # Copyright (c) 2023-2025 TANGAIR 
// # SPDX-License-Identifier: Apache-2.0

#include <stdint.h>

#ifdef __cplusplus
extern "C"{
#endif
#define STANDARD 0
#define EXTENDED 1
#pragma pack(1)
typedef struct FrameInfo {
    uint32_t canID;
    uint8_t frameType;
    uint8_t dataLength;
} FrameInfo;
#pragma pack()

int32_t openUSBCAN(const char *devName);

int32_t closeUSBCAN(int32_t dev);

int32_t sendUSBCAN(int32_t dev, uint8_t channel, FrameInfo* info, uint8_t *data);

int32_t readUSBCAN(int32_t dev, uint8_t *channel, FrameInfo* info, uint8_t *data, int32_t timeout);


#ifdef __cplusplus
}
#endif



// # Copyright (c) 2023-2025 TANGAIR 
// # SPDX-License-Identifier: Apache-2.0
