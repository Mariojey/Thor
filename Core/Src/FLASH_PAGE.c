
/**
  ***************************************************************************************************************
  ***************************************************************************************************************
  ***************************************************************************************************************
  File:	      FLASH_PAGE_F1.c
  Modifier:   ControllersTech.com
  Updated:    27th MAY 2021
  ***************************************************************************************************************
  Copyright (C) 2017 ControllersTech.com
  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.
  ***************************************************************************************************************
*/

#include "FLASH_PAGE.h"
#include "string.h"
#include "stdio.h"


/* STM32F103 have 128 PAGES (Page 0 to Page 127) of 1 KB each. This makes up 128 KB Flash Memory
 * Some STM32F103C8 have 64 KB FLASH Memory, so I guess they have Page 0 to Page 63 only.
 */

/* FLASH_PAGE_SIZE should be able to get the size of the Page according to the controller */
static uint32_t GetPage(uint32_t Address)
{
  for (int indx=0; indx<31; indx++)
  {
	  if((Address < (0x08000000 + (FLASH_PAGE_SIZE *(indx+1))) ) && (Address >= (0x08000000 + FLASH_PAGE_SIZE*indx)))
	  {
		  return (0x08000000 + FLASH_PAGE_SIZE*indx);
	  }
  }

  return 0;
}

uint8_t bytes_temp[4];


void float2Bytes(uint8_t * ftoa_bytes_temp,float float_variable)
{
    union {
      float a;
      uint8_t bytes[4];
    } thing;

    thing.a = float_variable;

    for (uint8_t i = 0; i < 4; i++) {
      ftoa_bytes_temp[i] = thing.bytes[i];
    }

}

float Bytes2float(uint8_t * ftoa_bytes_temp)
{
    union {
      float a;
      uint8_t bytes[4];
    } thing;

    for (uint8_t i = 0; i < 4; i++) {
    	thing.bytes[i] = ftoa_bytes_temp[i];
    }

   float float_variable =  thing.a;
   return float_variable;
}

uint32_t Flash_Write_Data_With_32_uint (uint32_t StartPageAddress, uint32_t *Data, uint16_t numberofwords)
{

	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError;
	int sofar=0;

	  /* Unlock the Flash to enable the flash control register access *************/
	   HAL_FLASH_Unlock();

	   /* Erase the user Flash area*/

	  uint32_t StartPage = GetPage(StartPageAddress);
	  uint32_t EndPageAdress = StartPageAddress + numberofwords*4;
	  uint32_t EndPage = GetPage(EndPageAdress);

	   /* Fill EraseInit structure*/
	   EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	   EraseInitStruct.Page = StartPage;
	   EraseInitStruct.NbPages     = ((EndPage - StartPage)/FLASH_PAGE_SIZE) +1;

	   if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	   {
	     /*Error occurred while page erase.*/
		  return HAL_FLASH_GetError ();
	   }

	   /* Program the user Flash area word by word*/

	   while (sofar<numberofwords)
	   {
	     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, StartPageAddress, Data[sofar]) == HAL_OK)
	     {
	    	 StartPageAddress += 4;  // use StartPageAddress += 2 for half word and 8 for double word
	    	 sofar++;
	     }
	     else
	     {
	       /* Error occurred while writing data in Flash memory*/
	    	 return HAL_FLASH_GetError ();
	     }
	   }

	   /* Lock the Flash to disable the flash control register access (recommended
	      to protect the FLASH memory against possible unwanted operation) *********/
	   HAL_FLASH_Lock();

	   return 0;
}
uint32_t Flash_Write_Data(uint32_t StartAddress, uint32_t *Data, uint16_t numberOfWords)
{
    HAL_FLASH_Unlock();

    uint32_t address = StartAddress;
    uint32_t buffer[4]; // bufor na 16 bajtów (4 x 32-bit)
    uint16_t i = 0;

    // Sprawdzenie wyrównania adresu do 16 bajtów
    if (address % 16 != 0)
    {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    while (i < numberOfWords)
    {
        // Wypełnij bufor danymi lub 0xFFFFFFFF, jeśli końcówka niepełna
        for (int j = 0; j < 4; j++)
        {
            if (i < numberOfWords)
                buffer[j] = Data[i++];
            else
                buffer[j] = 0xFFFFFFFF;  // padding
        }

        // Programowanie QUADWORD (128-bit = 16 bajtów)
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, address, (uint64_t*)buffer) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return HAL_FLASH_GetError();
        }

        address += 16; // skaczemy o 16 bajtów
    }

    HAL_FLASH_Lock();
    return HAL_OK;
}



void Flash_Read_Data_with_32_uint (uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords)
{
	while (1)
	{

		*RxBuf = *(__IO uint32_t *)StartPageAddress;
		StartPageAddress += 4;
		RxBuf++;
		if (!(numberofwords--)) break;
	}
}
void Flash_Read_Data(uint32_t StartAddress, uint32_t *RxBuf, uint16_t numberOfWords)
{
    for (int i = 0; i < numberOfWords; i++)
    {
        RxBuf[i] = *(__IO uint32_t *)(StartAddress + (i * 4));
    }
}
void Convert_To_Str (uint32_t *Data, char *Buf)
{
	int numberofbytes = ((strlen((char *)Data)/4) + ((strlen((char *)Data) % 4) != 0)) *4;

	for (int i=0; i<numberofbytes; i++)
	{
		Buf[i] = Data[i/4]>>(8*(i%4));
	}
}


void Flash_Write_NUM (uint32_t StartSectorAddress, float Num)
{

	float2Bytes(bytes_temp, Num);

	Flash_Write_Data (StartSectorAddress, (uint32_t *)bytes_temp, 1);
}


float Flash_Read_NUM (uint32_t StartSectorAddress)
{
	uint8_t buffer[4];
	float value;

	Flash_Read_Data(StartSectorAddress, (uint32_t *)buffer, 1);
	value = Bytes2float(buffer);
	return value;
}


void Flash_Erase_Page(uint32_t Address)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t pageError;

    eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.Page = (Address - FLASH_BASE) / FLASH_PAGE_SIZE;
    eraseInitStruct.NbPages = 1;
    eraseInitStruct.Banks = FLASH_BANK_2;

    if (HAL_FLASHEx_Erase(&eraseInitStruct, &pageError) != HAL_OK)
    {
        return HAL_FLASH_GetError();
    }

    HAL_FLASH_Lock();
}
