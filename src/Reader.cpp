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

#include "Reader.h"
#include "ErrorLogger.h"
#include "InputData.h"

#include <iostream>
#include <fstream>

#include "rapidjson/document.h"	
#include "rapidjson/filereadstream.h"	
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

struct DocumentObject
{
    DocumentObject()
    {
        m_document.SetNull();
    }

    rapidjson::Document m_document;
};

namespace
{
bool readDoubleFromObj(const rapidjson::Value& obj, std::string member, double* val)
{
    if (!obj.IsObject())
    {
        ErrLog::log("Error: input to readDoubleFromObj is not an object.");
        return false;
    }
    if (!obj.HasMember(member.c_str()))
    {
        ErrLog::log("Error: member " + member + "not found in object.");
        return false;
    }
    if (!(obj[member.c_str()].IsDouble() || obj[member.c_str()].IsInt64()))
    {
        ErrLog::log("Error: member " + member + "is not a double.");
        return false;
    }

    *val = obj[member.c_str()].GetDouble();

    return true;
}

bool readIntFromObj(const rapidjson::Value& obj, std::string member, int* val)
{
    if (!obj.IsObject())
    {
        ErrLog::log("Error: input to readDoubleFromObj is not an object.");
        return false;
    }
    if (!obj.HasMember(member.c_str()))
    {
        ErrLog::log("Error: member " + member + "not found in object.");
        return false;
    }
    if (!(obj[member.c_str()].IsDouble() || obj[member.c_str()].IsInt64()))
    {
        ErrLog::log("Error: member " + member + "is not a double.");
        return false;
    }

    *val = obj[member.c_str()].GetInt64();

    return true;
}
}


Reader::Reader()
{
    m_doc = std::make_unique<DocumentObject>();
}

Reader::~Reader() = default;

bool Reader::readInputData(std::string fileName, InputData* inputData)
{

    if (!parseFile(fileName))
    {
        return false;
    }

    if (!readSettings(&inputData->settings)) return false;

    if (m_doc->m_document.HasMember("concentration"))
    {
        if (m_doc->m_document["concentration"].IsDouble() || m_doc->m_document["concentration"].IsInt64())
        {
            inputData->concentration = m_doc->m_document["concentration"].GetDouble();
        }
    }

    if (m_doc->m_document.HasMember("domainSize"))
    {
        if (m_doc->m_document["domainSize"].IsArray())
        {
            const rapidjson::Value& sizeArr = m_doc->m_document["domainSize"];
            if (sizeArr.Size() != 2)
            {
                return false;
            }

            int i0(0), i1(1);
            inputData->domainSize[0] = sizeArr[i0].GetDouble();
            inputData->domainSize[1] = sizeArr[i1].GetDouble();
            
        }
    }

    if (!readBodyPoints(&inputData->bodyPointsVec))
    {
        ErrLog::log("Failed to read body points");
        return false;
    }


    int stop = 1;

    return true;
}


bool Reader::readSettings(Settings* settings)
{
    if (!m_doc->m_document.HasMember("settings")) return false;

    if (!m_doc->m_document["settings"].IsObject()) return false;

    const rapidjson::Value& settingsObj = m_doc->m_document["settings"];

    if (!readDoubleFromObj(settingsObj, "maxCollisionMargin", &settings->maxCollisionMargin)) return false;
    if (!readDoubleFromObj(settingsObj, "maxResidualPenetration", &settings->maxResidualPenetration)) return false;
    if (!readIntFromObj(settingsObj, "maxSolverIterations", &settings->maxSolverIterations)) return false;

    return true;
}


bool Reader::parseFile(std::string fileName)
{
    m_doc->m_document.SetNull();

    FILE* file;
    fopen_s(&file, fileName.c_str(), "r");

    if (file == NULL)
    {
        ErrLog::log("Error: Could not open input file.");
        return false;
    }

    char readBuffer[16384];
    rapidjson::FileReadStream inStream(file, readBuffer, sizeof(readBuffer));

    // Default template parameter uses UTF8 and MemoryPoolAllocator.
    m_doc->m_document.ParseStream<0>(inStream);

    if (m_doc->m_document.HasParseError())
    {
        ErrLog::log("Error: Failed to parse input file. " + fileName);
        return false;
    }

    fclose(file);

    return true;
}



bool Reader::readBodyPoints(std::vector<std::vector<Vector2>>* bodyPointsVec)
{
    if (m_doc->m_document.IsNull())
        return false;

    if (m_doc->m_document.HasMember("bodies"))
    {
        if (m_doc->m_document["bodies"].IsArray())
        {
            const rapidjson::Value& bodyArray = m_doc->m_document["bodies"];
            
            for (rapidjson::SizeType i = 0; i < bodyArray.Size(); i++)
            {
                if (bodyArray[i].IsObject())
                {
                    bool test = bodyArray[i].HasMember("points");

                    const rapidjson::Value& pointsArray = bodyArray[i]["points"];

                    if (pointsArray.Size() % 2 != 0) return false;

                    std::vector<Vector2> points;
                    for (rapidjson::SizeType j = 0; j < pointsArray.Size()/2; j++)
                    {
                        int idx = j * 2;
                        int idy = j * 2 + 1;

                        Vector2 p;
                        if (pointsArray[idx].IsDouble() || pointsArray[idx].IsInt64())
                        {
                            p.setX(pointsArray[idx].GetDouble());
                        }
                        
                        if (pointsArray[idy].IsDouble() || pointsArray[idx].IsInt64())
                        {
                            p.setY(pointsArray[idy].GetDouble());
                        }
                        points.push_back(p);
                    }

                    bodyPointsVec->push_back(points);
                }
            }

            return true;
        }
    }

    ErrLog::log("Warning: Failed to parse body points from the input file");
    return false;

}


