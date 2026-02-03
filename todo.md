# Alphaflight TODO
# Sammelsurium of firmware plans to implement when the board arrives

- Place DMA buffers (16 KiB RX / 16 KiB TX) in AXI SRAM (D1) and mark non-cacheable via MPU (update linker script & MPU settings)
- Use IDMA on the SDMMC peripheral and make a 100% register dependent driver (moving away from HAL for efficiency)
