ENTRY(Reset_Handler)
MEMORY
{
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 8K
  rom (rx)  : ORIGIN = 0x08000000, LENGTH = 128K
}
SECTIONS
{
    .  = 0x20000000;         /* From 0x08000000 */
    .text :
    {
        *(.text)      /* Program code */
        *(.rodata)    /* Read only data */
        *(vectors)    /* Vector table */
    } >ram

    .data :
    {
        *(.data)      /* Data memory */
    } >ram

    .bss :
    {
        _sbss = .;
        *(.bss)       /* Zero-filled run time allocate data memory */
        _ebss = .;
    } >ram
}
