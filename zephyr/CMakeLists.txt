if(CONFIG_TPM_TIS_SPI)
  zephyr_library(tpm-tis-spi)

  zephyr_include_directories(
    ../include
  )

  zephyr_library_sources(
    ../src/tpm-tis-spi.c
  )
endif()
