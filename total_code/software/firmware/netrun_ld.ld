/* netrun_ld.ld —— arena 整网执行器的链接脚本（.text.entry 必须在 0x8000） */
ENTRY(entry)
SECTIONS
{
  . = 0x8000;
  .text : {
    KEEP(*(.text.entry))
    *(.text .text.*)
  }
  .rodata : { *(.srodata .srodata.* .rodata .rodata.*) }
  .data :  { *(.sdata .sdata.* .data .data.*) }
  .bss (NOLOAD) : { *(.sbss .sbss.* .bss .bss.* COMMON) }
  /DISCARD/ : { *(.eh_frame .eh_frame.*) *(.comment) }
  /* 安全断言：镜像不得越过邮箱 0xFC00（2026-10-25 邮箱在 0x9F80，blob 长到 6.3KB
   * 曾盖住旧地址 0x9800 的邮箱；2026-10-29 加 NMS+FPS 后再次越界，
   * 邮箱整体搬到 arena 顶部 0xFC00，镜像可用空间 0x8000..0xFC00 = 31.5KB） */
  ASSERT(. <= 0xFC00, "arena image overlaps MAILBOX @0xFC00")
}

