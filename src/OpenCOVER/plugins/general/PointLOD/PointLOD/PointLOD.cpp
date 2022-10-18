/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#include <stdlib.h>
#include <inttypes.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <vector>
#include <algorithm>
#include <random>
#include <map>
#include <stdint.h>
#include <osg/Matrix>
#include <osg/Vec3>
#include <util/unixcompat.h>
#ifdef HAVE_E57
#include <e57/E57Foundation.h>
#include <e57/E57Simple.h>
#endif
#include <util/unixcompat.h>
#include <chrono>

#if defined(__GNUC__) && !defined(__clang__)
#include <parallel/algorithm>
namespace alg = __gnu_parallel;
#else
namespace alg = std;
#endif

using namespace std;

bool intensityOnly;
bool readScannerPositions = false;
uint32_t fileVersion=1;


struct Point
{
	float x, y, z;
    uint8_t r, g, b;
};


// ----------------------------------------------------------------------------
// readE57()
// ----------------------------------------------------------------------------
void readE57(char *filename, std::vector<Point> &vec)
{

#ifdef HAVE_E57

	osg::Matrix m;
	m.makeIdentity();
	try
	{
		e57::Reader	eReader(filename);
		e57::E57Root	rootHeader;
		eReader.GetE57Root(rootHeader);

		//Get the number of scan images available
		int data3DCount = eReader.GetData3DCount();
		e57::Data3D	scanHeader;
		cerr << "Total num of sets is " << data3DCount << endl;
		for (int scanIndex = 0; scanIndex < data3DCount; scanIndex++)
		{
			eReader.ReadData3D(scanIndex, scanHeader);
			osg::Matrix trans;
			trans.makeTranslate(scanHeader.pose.translation.x, scanHeader.pose.translation.y, scanHeader.pose.translation.z);
			osg::Matrix rot;
			rot.makeRotate(osg::Quat(scanHeader.pose.rotation.x, scanHeader.pose.rotation.y, scanHeader.pose.rotation.z, scanHeader.pose.rotation.w));
			m = rot*trans;

			int64_t nColumn = 0;
			int64_t nRow = 0;
			int64_t nPointsSize = 0;	//Number of points
			int64_t nGroupsSize = 0;	//Number of groups
			int64_t nCountSize = 0;		//Number of points per group
			bool	bColumnIndex = false; //indicates that idElementName is "columnIndex"

			eReader.GetData3DSizes(scanIndex, nRow, nColumn, nPointsSize, nGroupsSize, nCountSize, bColumnIndex);

			int64_t nSize = nRow;
			if (nSize == 0) nSize = 1024;	// choose a chunk size

            int8_t * isInvalidData = NULL;
            isInvalidData = new int8_t[nSize];
            if (!scanHeader.pointFields.cartesianInvalidStateField)
            {
                for (int i = 0; i < nSize; i++)
                    isInvalidData[i] = 0;
            }

			double * xData = NULL;
			if (scanHeader.pointFields.cartesianXField)
				xData = new double[nSize];
			double * yData = NULL;
			if (scanHeader.pointFields.cartesianYField)
				yData = new double[nSize];
			double * zData = NULL;
			if (scanHeader.pointFields.cartesianZField)
				zData = new double[nSize];
			double * rangeData = NULL;
			if (scanHeader.pointFields.sphericalRangeField)
				rangeData = new double[nSize];
			double * azData = NULL;
			if (scanHeader.pointFields.sphericalAzimuthField)
				azData = new double[nSize];
			double * elData = NULL;
			if (scanHeader.pointFields.sphericalElevationField)
				elData = new double[nSize];

			double *	intData = NULL;
			bool		bIntensity = false;
			double		intRange = 0;
			double		intOffset = 0;


			if (scanHeader.pointFields.intensityField)
			{
				bIntensity = true;
				intData = new double[nSize];
				intRange = scanHeader.intensityLimits.intensityMaximum - scanHeader.intensityLimits.intensityMinimum;
				intOffset = scanHeader.intensityLimits.intensityMinimum;
			}


			uint16_t *	redData = NULL;
			uint16_t *	greenData = NULL;
			uint16_t *	blueData = NULL;
			bool		bColor = false;
			int32_t		colorRedRange = 1;
			int32_t		colorRedOffset = 0;
			int32_t		colorGreenRange = 1;
			int32_t		colorGreenOffset = 0;
			int32_t		colorBlueRange = 1;
			int32_t		colorBlueOffset = 0;


			if (scanHeader.pointFields.colorRedField)
			{
				bColor = true;
				redData = new uint16_t[nSize];
				greenData = new uint16_t[nSize];
				blueData = new uint16_t[nSize];
				colorRedRange = scanHeader.colorLimits.colorRedMaximum - scanHeader.colorLimits.colorRedMinimum;
				colorRedOffset = scanHeader.colorLimits.colorRedMinimum;
				colorGreenRange = scanHeader.colorLimits.colorGreenMaximum - scanHeader.colorLimits.colorGreenMinimum;
				colorGreenOffset = scanHeader.colorLimits.colorGreenMinimum;
				colorBlueRange = scanHeader.colorLimits.colorBlueMaximum - scanHeader.colorLimits.colorBlueMinimum;
				colorBlueOffset = scanHeader.colorLimits.colorBlueMinimum;
			}


			int64_t * idElementValue = NULL;
			int64_t * startPointIndex = NULL;
			int64_t * pointCount = NULL;
			if (nGroupsSize > 0)
			{
				idElementValue = new int64_t[nGroupsSize];
				startPointIndex = new int64_t[nGroupsSize];
				pointCount = new int64_t[nGroupsSize];

				if (!eReader.ReadData3DGroupsData(scanIndex, nGroupsSize, idElementValue,
					startPointIndex, pointCount))
					nGroupsSize = 0;
			}

			int8_t * rowIndex = NULL;
			int32_t * columnIndex = NULL;
			if (scanHeader.pointFields.rowIndexField)
				rowIndex = new int8_t[nSize];
			if (scanHeader.pointFields.columnIndexField)
				columnIndex = new int32_t[nRow];


			e57::CompressedVectorReader dataReader = eReader.SetUpData3DPointsData(
				scanIndex,			//!< data block index given by the NewData3D
				nSize,				//!< size of each of the buffers given
				xData,				//!< pointer to a buffer with the x data
				yData,				//!< pointer to a buffer with the y data
				zData,				//!< pointer to a buffer with the z data
				isInvalidData,		//!< pointer to a buffer with the valid indication
				intData,			//!< pointer to a buffer with the lidar return intesity
				NULL,
				redData,			//!< pointer to a buffer with the color red data
				greenData,			//!< pointer to a buffer with the color green data
				blueData,			//!< pointer to a buffer with the color blue data
				NULL, //sColorInvalid
				rangeData,
				azData,
				elData
				/*rowIndex,			//!< pointer to a buffer with the rowIndex
				columnIndex			//!< pointer to a buffer with the columnIndex*/
			);

			int64_t		count = 0;
			unsigned	size = 0;
			int			col = 0;
			int			row = 0;
			Point point;
			while ((size = dataReader.read()))
			{
				for (unsigned int i = 0; i < size; i++)
				{

					if (isInvalidData[i] == 0 && xData)
					{
						osg::Vec3 p(xData[i], yData[i], zData[i]);
						p = p * m;
						point.x = p[0];
						point.y = p[1];
						point.z = p[2];

						point.r = ((redData[i] - colorRedOffset) * 255.0) / colorRedRange;
						point.g = ((greenData[i] - colorGreenOffset) * 255.0) / colorGreenRange;
						point.b = ((blueData[i] - colorBlueOffset) * 255.0) / colorBlueRange;

						vec.push_back(point);
					}
					else if (isInvalidData[i] == 0 && rangeData)
					{
						osg::Vec3 p(rangeData[i] * cos(elData[i]) * cos(azData[i]), rangeData[i] * cos(elData[i]) * sin(azData[i]), rangeData[i] * sin(elData[i]));
						p = p * m;
						point.x = p[0];
						point.y = p[1];
						point.z = p[2];

						//Normalize color to 0 - 255
						point.r = ((redData[i] - colorRedOffset) * 255.0) / colorRedRange;
						point.g = ((greenData[i] - colorGreenOffset) * 255.0) / colorGreenRange;
						point.b = ((blueData[i] - colorBlueOffset) * 255.0) / colorBlueRange;

						vec.push_back(point);
					}
				}
			}

			dataReader.close();

			delete[] isInvalidData;
			delete[] xData;
			delete[] yData;
			delete[] zData;
			delete[] rangeData;
			delete[] azData;
			delete[] elData;
			delete[] intData;
			delete[] redData;
			delete[] greenData;
			delete[] blueData;
			delete[] idElementValue;
			delete[] startPointIndex;
			delete[] pointCount;
			delete[] rowIndex;
			delete[] columnIndex;
		}

		eReader.Close();
		return;
		}
		catch (e57::E57Exception& ex) {
			ex.report(__FILE__, __LINE__, __FUNCTION__);
			return;
		}
		catch (std::exception& ex) {
			cerr << "Got an std::exception, what=" << ex.what() << endl;
			return;
		}
		catch (...) {
			cerr << "Got an unknown exception" << endl;
			return;
		}
#else
		cout << "Missing e75 library " << filename << endl;
#endif

}

// ----------------------------------------------------------------------------
// writeData()
// ----------------------------------------------------------------------------
void writeXYZ(const std::string& filename, std::vector<Point>& vec)
{
    std::ofstream xyz_file_stream(filename);

    if (!xyz_file_stream.is_open())
        throw std::runtime_error("Unable to open file: " +
            filename);

	for (int j = 0; j < vec.size(); ++j) {
        xyz_file_stream 
            << vec.at(j).x << " "
            << vec.at(j).y << " "
            << vec.at(j).z << " "
            << int(vec.at(j).r) << " "
            << int(vec.at(j).g) << " "
            << int(vec.at(j).b) << " \n";
    }

    xyz_file_stream.close();
}


// ----------------------------------------------------------------------------
// printHelpPage()
// ----------------------------------------------------------------------------
void printHelpPage()
{
    cout << endl;
    cout << "PointLOD - rearrange and filter point cloud data" << endl;
    cout << endl;
    cout << "Usage: PointLOD [options ...] inputfile outputfile" << endl;
    cout << endl;
    cout << "options" << endl;
	cout << "-c		-> converts .e57 to .xyz" << endl; 
    cout << endl;
    cout << "examples" << endl;
    cout << "PointLOD -c input.e57 output.xyz" << endl;
    cout << endl;
}

// ----------------------------------------------------------------------------
// main()
// ----------------------------------------------------------------------------
int main(int argc, char **argv)
{
	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    std::vector<Point> vec;
    std::map<int, int> lookUp;

    int nread = 0;
    intensityOnly=false;
    
    if (argc < 3) /* argc should be >= 3 for correct execution */
    {
        printf("error: minimal two params required\n");
        printHelpPage();
        
        return -1;
    }
	printf("argc: %i\n", argc);

	printf("argv[0]: %s\n", argv[0]);
	printf("argv[1]: %s\n", argv[1]);
	printf("argv[2]: %s\n", argv[2]);
	printf("argv[3]: %s\n", argv[3]);
	printf("argv[4]: %s\n", argv[4]);

	if (~argc == 5)	{ 
		return -1; 
	}
	if (!argv[2] == '-c') { 
		return -1; 
	}
	
	cout << "chosen option: conversion" << endl;
	
	char* inputfile = argv[3];

	if (int(strlen(inputfile)) > 4 && strcasecmp((inputfile + int(strlen(inputfile)) - 4), ".e57") == 0) {
		printf("reading file: %s\n", inputfile);
		std::chrono::steady_clock::time_point start_reading = std::chrono::steady_clock::now();
		printf("reading...\n");
		readE57(inputfile, vec);
		printf("done reading\n");
		std::chrono::steady_clock::time_point end_reading = std::chrono::steady_clock::now();
		std::cout << "Total time reading in s: " << std::chrono::duration_cast<std::chrono::seconds>(end_reading - start_reading).count() << std::endl;
	}
	char* ouputfile = argv[4];
	if (int(strlen(inputfile)) > 4 && strcasecmp((inputfile + int(strlen(inputfile)) - 4), ".e57") == 0) {
		printf("writing file: %s\n", ouputfile);
		std::chrono::steady_clock::time_point start_writing = std::chrono::steady_clock::now();
		printf("writing...\n");
		writeXYZ(ouputfile, vec);
		printf("done writing\n");
		std::chrono::steady_clock::time_point end_writing = std::chrono::steady_clock::now();
		std::cout << "Total time writing in s: " << std::chrono::duration_cast<std::chrono::seconds>(end_writing - start_writing).count() << std::endl;
	}
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "Total time in s: " << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << std::endl;

	return 0; 
}

// ----------------------------------------------------------------------------
