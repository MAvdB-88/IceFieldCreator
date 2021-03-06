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

#include <vector>

#include "InputData.h"
#include "Reader.h"
#include "Writer.h"
#include "CommandLineReader.h"
#include "IceFieldCreator.h"
#include "ErrorLogger.h"

int main(int argc, char* argv[])
{
    ErrLog errorLogger;

    InputData inputData;
    {
        CommandLineReader clReader;
        if (!clReader.readCommandLine(argc, argv))
        {
            return 1;
        }

        Reader reader;
        if (!reader.readInputData(clReader.inputFileName(), &inputData))
        {
            return 1;
        }
    }

    IceFieldCreator iceFieldCreator;
    if (!iceFieldCreator.createIceField(inputData))
    {
        return 1;
    }

    //Writes field to the current directory
    Writer::writeField("output.txt", iceFieldCreator.bodyVec(), inputData.domainSize); 

    return 0;
}
