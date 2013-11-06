        .section    .text.memmove,"ax",%progbits
        .align        2
        .global        memmove
        .thumb
        .thumb_func
        .type        memmove, %function
memmove:
    b   memcpy
.size        memmove, .-memmove
