.syntax unified
.cpu cortex-m3
.fpu softvfp
.thumb


  // *************** //
 //  Reset Handler  // 
// *************** //
.global reset_handler


.type reset_handler, %function
reset_handler:
    // initialize stack pointer
    ldr r0, =_estack
    mov sp, r0


    // copy data into RAM
    mov r0, 0
    ldr r1, =_sdata
    ldr r2, =_edata
    ldr r3, =_sidata
    b copy_sidata_loop

    copy_sidata:
        ldr r4, [r3, r0]
        str r4, [r1, r0]
        adds r0, r0, 4

    copy_sidata_loop:
        adds r4, r0, r1
        cmp r4, r2
        bne copy_sidata

    movs r0, 0
    ldr r1, =_sbss
    ldr r2, =_ebss
    b reset_bss_loop

    reset_bss:
        str r0, [r1]
        adds r1, r1, 4

    reset_bss_loop:
        cmp r1, r2
        bne reset_bss


    // C entry point
    b main
.size reset_handler, .-reset_handler

    
