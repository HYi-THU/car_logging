#pragma once

#include <iostream>
#include <unistd.h>
#include <cstring>
#include <numeric>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <vector>
#include <deque>

inline uint32_t convertStringToIntTime(const std::string &timeStr)
{
    std::tm tm = {};
    std::istringstream ss(timeStr);

    // 使用自定义格式读取日期和时间
    ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");

    // 检查是否成功解析时间字符串
    if (ss.fail())
    {
        throw std::runtime_error("Invalid time format");
    }

    // 将 tm 转换为 time_t
    std::time_t time = std::mktime(&tm);

    // 将 time_t 转换为 int，表示自UNIX纪元以来的秒数
    return static_cast<uint32_t>(time);
}

inline void str2vector(std::string &str, std::vector<double> &vec, double ratio)
{
    std::string tmp;
    for (int i = 0; i < str.size(); ++i)
    {
        if (str[i] == '/')
        {
            vec.emplace_back(stod(tmp) * ratio);
            tmp.clear();
        }
        else if (i == str.size() - 1)
        {
            tmp += str[i];
            vec.emplace_back(stod(tmp) * ratio);
            tmp.clear();
        }
        else
            tmp += str[i];
    }
}

inline std::vector<std::string> SplitStringByComma(const std::string &str)
{
    std::vector<std::string> tokens;
    std::istringstream iss(str);
    std::string token;

    while (std::getline(iss, token, ','))
    {
        tokens.push_back(token);
    }

    return tokens;
}

inline std::string ExtractNumbersAndDots(const std::string &str)
{
    std::string numbersAndDots;
    for (char c : str)
    {
        if (std::isdigit(c) || c == '.')
        {
            numbersAndDots += c;
        }
    }
    return numbersAndDots;
}



