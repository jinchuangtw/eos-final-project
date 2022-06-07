# cmake最低版本號要求
cmake_minimum_required(VERSION 3.0.0)

# 設定 PROJECT_NAME 變數
set(PROJECT_NAME PC_MCU_UART)

# 設定專案名稱
project (${PROJECT_NAME} CXX C)

# 設置 C/C++ 編譯選項
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")  # 設置 C++ 的編譯選項 
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")  # 設置 C 的編譯選項

# 設定 lib 路徑
set(LIB_DIR ${PROJECT_SOURCE_DIR}/lib)

# 查詢當前目錄下的所有原始檔並存入 DIR_SRCS 與 DIR_INCS 變數
aux_source_directory(./src DIR_SRCS)
aux_source_directory(./include/UART/RS232 DIR_INCS_RS232)
aux_source_directory(./include/Attitude_Kinematics DIR_INCS_Att)

# 設定 MSVC 編譯編碼 (必須位於 add_executable( ... ) 之前)
add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

# 設定專案與 gcc lib 的連結方式
if(MINGW)
    set(CMAKE_EXE_LINKER_FLAGS "-static")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "-s")
endif()

# 新增一個可編譯的目標到專案 (將 ${DIR_SRCS} 與 ${DIR_INCS} 路徑中所有的原始檔鏈結，並產生可執行檔 ${PROJECT_NAME}.exe)
add_executable(${PROJECT_NAME} ${DIR_SRCS} ${DIR_INCS_RS232} ${DIR_INCS_Att})

# 設定專案要產生連結的 libxxxx 檔案與其對應標頭檔所在路徑 (要在 add_executable 之後)
# target_include_directories(${PROJECT_NAME} PUBLIC ${LIB_DIR}/include) # 專案引入 lib 標頭檔路徑
# target_link_libraries(${PROJECT_NAME} ${LIB_DIR}/libdll/libCppFunc.dll) # 專案連結的 libxxxx 檔案 (連結動態程式庫) (要注意雖然可執行檔有連結程式庫，但最後還是要將 .dll 檔跟 .exe 可執行檔放在同一個路徑之下)
# target_link_libraries(${PROJECT_NAME} ${LIB_DIR}/liba/libCFunc.a) # 專案連結的 libxxxx 檔案 (連結靜態程式庫)

# 設定編譯後，可執行檔的產生位置: .\build\Debug
# (https://stackoverflow.com/questions/50301919/cmake-add-executable-in-another-directory)
set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/app")

# 設定工作目錄
# set_property(TARGET ${PROJECT_NAME} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/build/Debug)

# # 複製指定路徑所有檔案到可執行檔 (.exe) 的工作目錄
# add_custom_command(TARGET ${PROJECT_NAME} PRE_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory ${LIB_DIR}/libdll $<TARGET_FILE_DIR:${PROJECT_NAME}>)

#####
# 顯示 src 跟 include 這兩個目錄之下所有的 source code
foreach(file IN LISTS DIR_SRCS DIR_INCS_RS232)
message(STATUS ${file})
endforeach(file)