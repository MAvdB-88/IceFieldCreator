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

#include "CommandLineReader.h"

#include <Windows.h>
#include <iostream>
#include <fstream>

bool CommandLineReader::readCommandLine(int argc, char* argv[])
{
	if (argc < 2)
	{
		if (!checkFileExistance())
		{
			return false;
		}
	}
	else if (argc == 2)
	{
		m_intputFileName = argv[1];
		if (!checkFileExistance())
		{
			return false;
		}
	}

	return true;
}


bool CommandLineReader::checkFileExistance()
{
	bool validFileName(false);
	int cnt(0);
	while (!validFileName && cnt < 5)
	{
		std::ifstream f(m_intputFileName);
		if (f.good())
		{
			validFileName = true;
			break;
		}
		cnt++;

		std::cout << "Failed to open the config file. Please specify the input file path: " << std::endl;
		std::string str;
		std::cin >> m_intputFileName;
	}

	if (!validFileName)
	{
		std::cout << "Failed to open the input file. Press enter to exit" << std::endl;
		std::cin.ignore(256, '\n');
		std::cin.get();
	}

	return validFileName;
}