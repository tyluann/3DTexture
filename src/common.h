#pragma once

#include <cstdio>

#define loop(i, n) for(int i = 0; i < n; ++i)

#define LOGI(format, ...) printf("[Log Info] [File:%s, Line:%d] "format"\n", __FILE__, __LINE__, ##__VA_ARGS__)
#define LOGW(format, ...) printf("[Log Warning] [File:%s, Line:%d] "format"\n", __FILE__, __LINE__, ##__VA_ARGS__)
#define LOGE(format, ...) printf("[Log Error] [File:%s, Line:%d] "format"\n", __FILE__, __LINE__, ##__VA_ARGS__)


//constexpr auto AA = 0;