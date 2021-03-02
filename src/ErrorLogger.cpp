// MIT License

// Copyright (c) 2020 Marnix van den Berg <m.a.vdberg88@gmail.com>

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "ErrorLogger.h"
#include <Windows.h>
#include <iostream>


std::ofstream  ErrLog::sg_logFile;

ErrLog::ErrLog()
{
    if (!sg_logFile.is_open())
    {
        char buffer[MAX_PATH];
        GetCurrentDirectory(sizeof(buffer), buffer);

        std::string str = buffer;
        str += "\\logFile.txt";

        sg_logFile.open(str);
    }
    else
    {
        std::string str = "Warning: Trying to re-initialize already open log file";
        log(str);
    }
}


void  ErrLog::log(std::string message)
{ 
    //Write to console and to log file:
    std::cout << message << std::endl;
    sg_logFile << message << std::endl;
}