        .section    .text.memset,"ax",%progbits
        .align        2
        .global        memset
        .thumb
        .thumb_func
        .type        memset, %function
memset:
    bx  lr
.size        memset, .-memset

