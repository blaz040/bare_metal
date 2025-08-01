.syntax unified
.cpu cortex-m4
.thumb

.global reset_handler
.type reset_handler, %function
reset_handler:
  LDR  r0, =_estack
  MOV  sp, r0
  // Copy data from flash to RAM data init section.
  // R2 will store our progress along the sidata section.
  MOVS r0, #0
  // Load the start/end addresses of the data section,
  // and the start of the data init section.
  LDR  r1, =_sdata
  LDR  r2, =_edata
  LDR  r3, =_sidata
  B    copy_sidata_loop
copy_sidata:
  // Offset the data init section by our copy progress.
  LDR  r4, [r3, r0]
  // Copy the current word into data, and increment.
  STR  r4, [r1, r0]
  ADDS r0, r0, #4
copy_sidata_loop:
  // Unless we've copied the whole data section, copy the
  // next word from sidata->data.
  ADDS r4, r0, r1
  CMP  r4, r2
  BCC  copy_sidata
  // Once we are done copying the data section into RAM,
  // move on to filling the BSS section with 0s.
  MOVS r0, #0
  LDR  r1, =_sbss
  LDR  r2, =_ebss
  B    reset_bss_loop
  // Fill the BSS segment with '0's.
reset_bss:
  // Store a 0 and increment by a word.
  STR  r0, [r1]
  ADDS r1, r1, #4
reset_bss_loop:
  // We'll use R1 to count progress here; if we aren't
  // done, reset the next word and increment.
  CMP  r1, r2
  BCC  reset_bss
  B    main
.size reset_handler, .-reset_handler
