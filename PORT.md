## Porting To STM32F429

## Templates
### Linker script
- gnu_id
    - cortex
        - armv7em
            - stm32f429
                - memory_map
                - script_specific
                - script
After generation using goil these three files will generate one file called script.ld

### Config
#### config.oil

FSMC_IRQHandler -> FMC_IRQHandler
Added: 
UART7_IRQHandler   
UART8_IRQHandler   
SPI4_IRQHandler    
SPI5_IRQHandler    
SPI6_IRQHandler    
SAI1_IRQHandler    
LTDC_IRQHandler    
LTDC_ER_IRQHandler 
DMA2D_IRQHandler   

```
  INTERRUPT_COUNT nb_it {
    IT_TABLE_SIZE = 107;
  };
```
## Machine
- cortex
    - armv7em
        - stm32f429
Registers definitions and Includes (Generic for all STM32F4 family)

