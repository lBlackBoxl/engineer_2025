#include "main.h"
#include "STS_drive.h"
#include <stdlib.h>

uint8_t wBuf[128];
uint8_t wLen = 0;
static uint8_t u8Status;//鑸垫満鐘舵€�
static uint8_t u8Error;//閫氫俊鐘舵€�

enum SCS_ERR_LIST
{
    SCS_ERR_NO_REPLY = 1,
    SCS_ERR_CRC_CMP = 2,
    SCS_ERR_SLAVE_ID = 3,
    SCS_ERR_BUFF_LEN = 4,
};

// 鎺ユ敹缂撳啿鍖哄埛鏂�
void read_FlushSCS()
{
    HAL_Delay(1);
}

// 鍙戦€佺紦鍐插尯鍒锋柊
void write_FlushSCS(UART_HandleTypeDef *huart)
{
    if (wLen)
    {
        HAL_UART_Transmit(huart, wBuf, wLen, 100);
        wLen = 0;
    }
}

// 鍚戝彂閫佺紦鍐插尯鍐呭啓鍏ユ暟鎹�
int writeSCS(unsigned char *nDat, int nLen)
{
    for (int i = 0; i < nLen; i++)
    {
        if (wLen < sizeof(wBuf))
        {
            wBuf[wLen] = *nDat;
            wLen++;
            nDat++;
        }
    }
    return wLen;
}

// 鍚戞寚瀹氱紦鍐插尯鎺ユ敹鏁版嵁
int readSCS(UART_HandleTypeDef *huart, unsigned char *nDat, int nLen)
{
    if (HAL_OK != HAL_UART_Receive(huart, nDat, nLen, 100))
    {
        return 0;
    }
    else
    {
        return nLen;
    }
}

// 妫€鏌ュ寘澶�
int checkHead(UART_HandleTypeDef *huart)
{
    uint8_t bDat;
    uint8_t bBuf[2] = {0, 0};
    uint8_t Cnt = 0;
    while (1)
    {
        if (!readSCS(huart, &bDat, 1))
        {
            return 0;
        }
        bBuf[1] = bBuf[0];
        bBuf[0] = bDat;
        if (bBuf[0] == 0xff && bBuf[1] == 0xff)
        {
            break;
        }
        Cnt++;
        if (Cnt > 10)
        {
            return 0;
        }
    }
    return 1;
}

void writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun)
{
    uint8_t i;
    uint8_t msgLen = 2;
    uint8_t bBuf[6];
    uint8_t CheckSum = 0;

    bBuf[0] = 0xff;
    bBuf[1] = 0xff;
    bBuf[2] = ID;
    bBuf[4] = Fun;
		if(nDat){
			msgLen += nLen + 1;
			bBuf[3] = msgLen;
			bBuf[5] = MemAddr;
			writeSCS(bBuf, 6);
			
		}else{
			bBuf[3] = msgLen;
			writeSCS(bBuf, 5);
		}

    CheckSum = ID + msgLen + Fun + MemAddr;
    if (nDat)
    {
        for (i = 0; i < nLen; i++)
        {
            CheckSum += nDat[i];
        }
        writeSCS(nDat, nLen);
    }

    CheckSum = ~CheckSum;
    writeSCS(&CheckSum, 1);
}

//璇诲彇鐢垫満宸ヤ綔鐘舵€�
//杩斿洖鍊间负宸ヤ綔鐘舵€�
uint8_t STS_CMD_PING(UART_HandleTypeDef *huart, uint8_t STS_ID)
{
    //鍙戦€佹寚浠�
    read_FlushSCS();
    writeBuf(STS_ID, 0, NULL, 0, STS_PING);
    write_FlushSCS(huart);

    //鎺ユ敹鏁版嵁甯�
    uint8_t bBuf[4];
    uint8_t calSum;
    u8Error = 0;
    u8Status = 0;
    if (!checkHead(huart))
    {
        u8Error = SCS_ERR_NO_REPLY;
        return 0;
    }
    if (readSCS(huart, bBuf, 4) != 4)
    {
        u8Error = SCS_ERR_NO_REPLY;
        return 0;
    }
    if (bBuf[0] != STS_ID && STS_ID != 0xfe)
    {
        u8Error = SCS_ERR_SLAVE_ID;
        return 0;
    }
    if (bBuf[1] != 2)
    {
        u8Error = SCS_ERR_BUFF_LEN;
        return 0;
    }
    calSum = ~(bBuf[0] + bBuf[1] + bBuf[2]);
    if (calSum != bBuf[3])
    {
        u8Error = SCS_ERR_CRC_CMP;
        return 0;
    }
    u8Status = bBuf[2];
    return u8Status;
}

//浠庤埖鏈哄唴瀛樹腑璇诲嚭鏁版嵁
//杩斿洖鍊间负鏁版嵁
uint16_t STS_CMD_READ(UART_HandleTypeDef *huart, uint8_t STS_ID, uint8_t MemAddr, uint8_t rLen)
{
    //鍙戦€佹寚浠�
    uint8_t tBuf[2] = {MemAddr, rLen};
    read_FlushSCS();
    writeBuf(STS_ID, MemAddr, tBuf, 2, STS_READ);
    write_FlushSCS(huart);

    //鎺ユ敹鏁版嵁甯�
		uint8_t* rBuf = (uint8_t*)malloc((rLen+4) * sizeof(uint8_t));
//		if(rBuf == NULL)
//		{
//			u8Error = SCS_ERR_BUFF_LEN;
//			return 0;
//		}
//    uint8_t rBuf[4 + rLen];
    uint8_t calSum;
    uint8_t i;
    uint16_t rData;
    u8Error = 0;
    if (!checkHead(huart))
    {
        u8Error = SCS_ERR_NO_REPLY;
        return 0;
    }
    if (readSCS(huart, rBuf, 4 + rLen) != (4 + rLen))
    {
        u8Error = SCS_ERR_NO_REPLY;
        return 0;
    }
    if (rBuf[0] != STS_ID && STS_ID != 0xfe)
    {
        u8Error = SCS_ERR_SLAVE_ID;
        return 0;
    }
    if (rBuf[1] != (rLen + 2))
    {
        u8Error = SCS_ERR_BUFF_LEN;
        return 0;
    }
    calSum = rBuf[0] + rBuf[1] + rBuf[2];
    for (i = 0; i < rLen; i++)
    {
        calSum += rBuf[i + 3];
    }
    calSum = ~calSum;
    if (calSum != rBuf[rLen + 3])
    {
        u8Error = SCS_ERR_CRC_CMP;
        return 0;
    }
    u8Status = rBuf[2];
    if(rLen == 1)
    {
        rData = rBuf[3];
    }
    else if(rLen == 2)
    {
        rData = rBuf[3] + (rBuf[4] << 8);
    }
		
		free(rBuf);
    return rData;
}

//鍚戣埖鏈哄唴瀛樹腑鍐欏叆鏁版嵁
void STS_CMD_WRITE(UART_HandleTypeDef *huart, uint8_t STS_ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
    //鍙戦€佹寚浠�
    read_FlushSCS();
    writeBuf(STS_ID, MemAddr, nDat, nLen, STS_WRITE);
    write_FlushSCS(huart);
}

//寮傛�ュ啓鎸囦护
void STS_CMD_ASYNC_WRITE(UART_HandleTypeDef *huart, uint8_t STS_ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
    //鍙戦€佹寚浠�
    read_FlushSCS();
    writeBuf(STS_ID, MemAddr, nDat, nLen, STS_REG_WRITE);
    write_FlushSCS(huart);
}

//鎵ц�屽紓姝ュ啓鎸囦护
void STS_CMD_ASYNC_ACTION(UART_HandleTypeDef *huart, uint8_t STS_ID)
{
    //鍙戦€佹寚浠�
    read_FlushSCS();
    writeBuf(STS_ID, 0, NULL, 0, STS_ACTION);
    write_FlushSCS(huart);
}

//鍚屾�ュ啓鎸囦护
void STS_CMD_SYNC_WRITE(UART_HandleTypeDef *huart, uint8_t STS_ID[],uint8_t IDN, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen)
{
    uint8_t mesLen = ((nLen+1) * IDN + 4);
    uint8_t Sum = 0;
    uint8_t bBuf[7];
    uint8_t i, j;

    bBuf[0] = 0xff;
    bBuf[1] = 0xff;
    bBuf[2] = 0xfe;
    bBuf[3] = mesLen;
    bBuf[4] = STS_SYCNWRITE;
    bBuf[5] = MemAddr;
    bBuf[6] = nLen;

    read_FlushSCS();
    writeSCS(bBuf, 7);

    Sum = 0xfe + mesLen + STS_SYCNWRITE + MemAddr + nLen;

    for (i = 0; i < IDN; i++)
    {
        writeSCS(&STS_ID[i], 1);
        writeSCS(nDat + i * nLen, nLen);
        Sum += STS_ID[i];
        for (j = 0; j < nLen; j++)
        {
            Sum += nDat[i * nLen + j];
        }
    }
    Sum = ~Sum;
    writeSCS(&Sum, 1);
    write_FlushSCS(huart);
}

//鍚屾�ヨ�绘寚浠�
uint8_t STS_CMD_SYNC_READ(UART_HandleTypeDef *huart, uint8_t STS_ID[], uint8_t IDN, uint8_t MemAddr,uint8_t *rData[], uint8_t rLen)
{
    //鍙戦€佹寚浠�
    uint8_t mesLen = (IDN + 4);
    uint8_t Sum = 0;
    uint8_t bBuf[7];

    bBuf[0] = 0xff;
    bBuf[1] = 0xff;
    bBuf[2] = 0xfe;
    bBuf[3] = mesLen;
    bBuf[4] = STS_SYCNREAD;
    bBuf[5] = MemAddr;
    bBuf[6] = rLen;

    read_FlushSCS();
    writeSCS(bBuf, 7);

    Sum = 0xfe + mesLen + STS_SYCNREAD + MemAddr + rLen;
    for (int i = 0; i < IDN; i++)
    {
        writeSCS(&STS_ID[i], 1);
        Sum += STS_ID[i];
    }
    Sum = ~Sum;
    writeSCS(&Sum, 1);
    write_FlushSCS(huart);

    //鎺ユ敹鏁版嵁甯�
		uint8_t* rBuf = (uint8_t*)malloc((rLen+4) * sizeof(uint8_t));
//		if(rBuf == NULL)
//		{
//			u8Error = SCS_ERR_BUFF_LEN;
//			return 0;
//		}
//    uint8_t rBuf[4 + rLen];
    u8Error = 0;
		u8Status = 0;
		uint8_t calSum;
    for (int i = 0; i < IDN; i++)
    {
        if (!checkHead(huart))
        {
            u8Error = SCS_ERR_NO_REPLY;
            return 0;
        }
        if (readSCS(huart, rBuf, 4 + rLen) != (4 + rLen))
        {
            u8Error = SCS_ERR_NO_REPLY;
            return 0;
        }
        if (bBuf[0] != STS_ID[i] && STS_ID[i] != 0xfe)
        {
            u8Error = SCS_ERR_SLAVE_ID;
            return 0;
        }
        if (bBuf[1] != (rLen + 2))
        {
            u8Error = SCS_ERR_BUFF_LEN;
            return 0;
        }
        calSum = bBuf[0] + bBuf[1] + bBuf[2];
        for (i = 0; i < rLen; i++)
        {
            calSum += bBuf[i + 3];
        }
        calSum = ~calSum;
        if (calSum != bBuf[rLen + 3])
        {
            u8Error = SCS_ERR_CRC_CMP;
            return 0;
        }
        for (int j = 0; j < rLen; j++)
        {
            rData[i][j] = bBuf[j + 3];
        }
    }
		u8Status = bBuf[2];
		free(rBuf);
		return  u8Status;
}

uint8_t STS_CMD_RECOVERY(UART_HandleTypeDef *huart, uint8_t STS_ID)
{
    //鍙戦€佹寚浠�
    read_FlushSCS();
    writeBuf(STS_ID, 0, NULL, 0, STS_RECOVERY);
    write_FlushSCS(huart);

    // 鎺ユ敹鏁版嵁甯�
    uint8_t bBuf[4];
    uint8_t calSum;
    u8Error = 0;
    u8Status = 0;
    if (!checkHead(huart))
    {
        u8Error = SCS_ERR_NO_REPLY;
        return 0;
    }
    if (readSCS(huart, bBuf, 4) != 4)
    {
        u8Error = SCS_ERR_NO_REPLY;
        return 0;
    }
    if (bBuf[0] != STS_ID && STS_ID != 0xfe)
    {
        u8Error = SCS_ERR_SLAVE_ID;
        return 0;
    }
    if (bBuf[1] != 2)
    {
        u8Error = SCS_ERR_BUFF_LEN;
        return 0;
    }
    calSum = ~(bBuf[0] + bBuf[1] + bBuf[2]);
    if (calSum != bBuf[3])
    {
        u8Error = SCS_ERR_CRC_CMP;
        return 0;
    }
    u8Status = bBuf[2];
    return u8Status;
}

uint8_t STS_CMD_RESET(UART_HandleTypeDef *huart, uint8_t STS_ID)
{
    //鍙戦€佹寚浠�
    read_FlushSCS();
    writeBuf(STS_ID, 0, NULL, 0, STS_RESET);
    write_FlushSCS(huart);

    // 鎺ユ敹鏁版嵁甯�
    uint8_t bBuf[4];
    uint8_t calSum;
    u8Error = 0;
    u8Status = 0;
    if (!checkHead(huart))
    {
        u8Error = SCS_ERR_NO_REPLY;
        return 0;
    }
    if (readSCS(huart, bBuf, 4) != 4)
    {
        u8Error = SCS_ERR_NO_REPLY;
        return 0;
    }
    if (bBuf[0] != STS_ID && STS_ID != 0xfe)
    {
        u8Error = SCS_ERR_SLAVE_ID;
        return 0;
    }
    if (bBuf[1] != 2)
    {
        u8Error = SCS_ERR_BUFF_LEN;
        return 0;
    }
    calSum = ~(bBuf[0] + bBuf[1] + bBuf[2]);
    if (calSum != bBuf[3])
    {
        u8Error = SCS_ERR_CRC_CMP;
        return 0;
    }
    u8Status = bBuf[2];
    return u8Status;
}

// 浣嶇疆鏍″噯
uint8_t STS_CMD_POS_ADJ(UART_HandleTypeDef *huart, uint8_t STS_ID, uint16_t target_position)
{
    // 鍙戦€佹寚浠�
    read_FlushSCS();
    uint8_t tBuf[2] = {(uint8_t)(target_position & 0x00FF), (uint8_t)(target_position >> 8)};
    writeBuf(STS_ID, 0, tBuf, 2, STS_POS_ADJ);
    write_FlushSCS(huart);
    // 鎺ユ敹鏁版嵁甯�
    uint8_t bBuf[4];
    uint8_t calSum;
    u8Error = 0;
    u8Status = 0;
    if (!checkHead(huart))
    {
        u8Error = SCS_ERR_NO_REPLY;
        return 0;
    }
    if (readSCS(huart, bBuf, 4) != 4)
    {
        u8Error = SCS_ERR_NO_REPLY;
        return 0;
    }
    if (bBuf[0] != STS_ID && STS_ID != 0xfe)
    {
        u8Error = SCS_ERR_SLAVE_ID;
        return 0;
    }
    if (bBuf[1] != 2)
    {
        u8Error = SCS_ERR_BUFF_LEN;
        return 0;
    }
    calSum = ~(bBuf[0] + bBuf[1] + bBuf[2]);
    if (calSum != bBuf[3])
    {
        u8Error = SCS_ERR_CRC_CMP;
        return 0;
    }
    u8Status = bBuf[2];
    return u8Status;
}
