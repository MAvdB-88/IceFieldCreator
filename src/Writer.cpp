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

#include "Writer.h"
#include "ErrorLogger.h"

#include "Body.h"
#include "BodyVec.h"

#include "rapidjson/document.h"	
#include "rapidjson/filewritestream.h"	
#include "rapidjson/stringbuffer.h"
#include "rapidjson/PrettyWriter.h"


bool Writer::writeField(std::string fileName, const BodyVec& bodies, Vector2 domainSize)
{
    rapidjson::Document document;
    rapidjson::Document::AllocatorType& allocator = document.GetAllocator();

    document.SetObject();

    //Write the domain size:
    rapidjson::Value domainSizeArr(rapidjson::kArrayType);
    domainSizeArr.PushBack(domainSize.x(), allocator);
    domainSizeArr.PushBack(domainSize.y(), allocator);
    document.AddMember("domainSize", domainSizeArr, allocator);

    //Write the bodies:
    rapidjson::Value bodyArray(rapidjson::kArrayType);

    for (int i = 0; i < bodies.size(); i++)
    {
        const Body& body = bodies[i];

        if (body.isStatic()) continue; //Don't write the walls

        rapidjson::Value bodyObj(rapidjson::kObjectType);
        bodyObj.AddMember("orientation", body.transform().orientation().angle(), allocator);

        Vector2 position = body.transform().position();
        rapidjson::Value posArr(rapidjson::kArrayType);
        posArr.PushBack(position.x(), allocator);
        posArr.PushBack(position.y(), allocator);
        bodyObj.AddMember("position", posArr, allocator);


        rapidjson::Value shapeArray(rapidjson::kArrayType);
        const std::vector<Vector2>& shapePoints = body.localPoints();
        for (int i = 0; i < shapePoints.size(); i++)
        {
            Vector2 p = shapePoints[i];
            shapeArray.PushBack(p.x(), allocator);
            shapeArray.PushBack(p.y(), allocator);
        }
        bodyObj.AddMember("shape", shapeArray, allocator);

        bodyArray.PushBack(bodyObj, allocator);
    }

    document.AddMember("bodies", bodyArray, allocator);

    FILE* file;
    fopen_s(&file, fileName.c_str(), "w");

    char writeBuffer[16384];
    rapidjson::FileWriteStream os(file, writeBuffer, sizeof(writeBuffer));

    rapidjson::PrettyWriter<rapidjson::FileWriteStream> writer(os);
    
    writer.SetFormatOptions(rapidjson::PrettyFormatOptions::kFormatSingleLineArray);

    document.Accept(writer);

    return true;
}