# Document Title

For CMake toolchain
https://www.qnx.com/developers/docs/qnxeverywhere/com.qnx.doc.qpg/topic/port_CMake.html

pass as `-DCMAKE_TOOLCHAIN_PATH` 

Set the default, helps to avoid compiling to x86
```
qcc -Vgcc_ntoaarch64le -set-default
```
