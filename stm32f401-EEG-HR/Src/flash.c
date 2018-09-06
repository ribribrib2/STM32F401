


void writeFlashTest(void)
{
	  //½âËøflash
		HAL_FLASH_Unlock();
	  //
    FLASH_EraseInitTypeDef f;

    f.TypeErase = FLASH_TYPEERASE_PAGES;

    f.PageAddress = addr;

    f.NbPages = 1;

    uint32_t PageError = 0;

    //??????

    HAL_FLASHEx_Erase(&f, &PageError);



    //3??FLASH??

    HAL_FLASH_Program(TYPEPROGRAM_WORD, addr, writeFlashData);



    //4???FLASH

  HAL_FLASH_Lock();

}