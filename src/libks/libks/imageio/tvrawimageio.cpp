// This file was written by Toby Vaudrey
// Published on http://www.cs.auckland.ac.nz/~tvau003/CodeExamples/tvImageIO.zip

#include "tvrawimageio.h"

#include <typeinfo>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <cstring>
//#include <typeinfo.h>
typedef short int word;

using namespace std;

//////////////////////////////////////////////////////////////////////////////
// Write PGM Methods
//////////////////////////////////////////////////////////////////////////////

bool TvRawImageIO::writePGM(char *file_out_name, 
							TvImageSize & imageSize,
							char * src, 
							const bool bigEndian )
{
	ofstream file_out;
	file_out.open ( file_out_name, fstream::binary | fstream::out );

	if ( !file_out )
	{
		cout << "writePGM:  Cannot open the file " << file_out_name << "\n";
		return true;
	}

	// Write header
	if (imageSize.dataType == TV_DATATYPE_8U)
		writePgmHeader(file_out, imageSize, MAX_8U);
	else if (imageSize.dataType == TV_DATATYPE_16U)
		writePgmHeader(file_out, imageSize, MAX_16U);
	else
	{
		cout << "writePGM:  Data type not supported \n";
		return true;
	}

	bool byteSwap = 
		(imageSize.dataType == TV_DATATYPE_16U) && bigEndian;

	// Write the ground truth data
	if ( writeData( file_out, imageSize, src, byteSwap) )
	{
		cout << "writePGM: output file failed to write \n";
		file_out.close();
		return true;
	}

	file_out.close();
	return false;
}

//////////////////////////////////////////////////////////////////////////////
// Load PGM Methods
//////////////////////////////////////////////////////////////////////////////


bool TvRawImageIO::loadPGM(char *file_in_name, 
						   TvImageSize & imageSize,
						   char * imageData,
						   const bool bigEndian  )

{
	// Set up file
	ifstream file_in;
	file_in.open ( file_in_name, std::ios::binary );
	
	if ( !file_in )
	{
		cout << "loadPGM:  Cannot open the input file: " << file_in_name << "\n";
		return true;
	}

	// Load data from header
	TvImageSize tempImageSize;
	if ( loadPGMfileHeader(file_in, tempImageSize) )
	{
		file_in.close();
		return true;
	}

	// Load the image if headers match
	if (tempImageSize == imageSize)
	{
		bool error =
			loadPGMdata(file_in, imageSize, imageData, bigEndian);
		if (error)
		{
			file_in.close();
			return true;
		}
	}
	else
	{
		cout << "loadPGM: input image does not match input file \n";
		file_in.close();
		return true;
	}

	file_in.close();
	return false;
}

char * TvRawImageIO::loadPGM(char *file_in_name, 
							 TvImageSize & imageSize,
							 const bool bigEndian)
{
	char * outImage = NULL;

	// Set up file
	ifstream file_in;
	file_in.open ( file_in_name, std::ios::binary );
	
	if ( !file_in )
	{
		cout << "loadPGM:  Cannot open the input file: " << file_in_name << "\n";
		return NULL;
	}

	// Load data from header
	if ( loadPGMfileHeader(file_in, imageSize) )
	{
		file_in.close();
		return NULL;
	}

	outImage = new char[imageSize.getNumberBytes()];

	if ( loadPGMdata(file_in, imageSize, outImage, bigEndian) )
	{
		file_in.close();
		return NULL;
	}

	file_in.close();

	return outImage;

}


//////////////////////////////////////////////////////////////////////////////
// Load PPM Methods
//////////////////////////////////////////////////////////////////////////////

char * TvRawImageIO::loadPPM(char *file_in_name, 
							 TvImageSize & imageSize,
							 const bool bigEndian)
{
	char * outImage = NULL;

	// Set up file
	ifstream file_in;
	file_in.open ( file_in_name, std::ios::binary );
	
	if ( !file_in )
	{
		cout << "loadPPM:  Cannot open the input file: " << file_in_name << "\n";
		return NULL;
	}

	// Load data from header
	if ( loadPPMfileHeader(file_in, imageSize) )
	{
		file_in.close();
		return NULL;
	}

	outImage = new char[imageSize.getNumberBytes()];

	if ( loadPGMdata(file_in, imageSize, outImage, bigEndian) )
	{
		file_in.close();
		return NULL;
	}

	file_in.close();

	return outImage;

}



//////////////////////////////////////////////////////////////////////////////
// Load Raw Data Methods
//////////////////////////////////////////////////////////////////////////////


char * TvRawImageIO::loadRawDataImage(char *file_in_name, TvImageSize & imageSize)
{
	char * outImage = NULL;

	// Set up file
	ifstream file_in;
	file_in.open ( file_in_name, std::ios::binary );
	
	if ( !file_in )
	{
		cout << "loadRawDataImage:  Cannot open the input file: " << file_in_name << "\n";
		return NULL;
	}

	// Load data from header
	if ( loadRawDataFileHeader(file_in, imageSize) )
	{
		file_in.close();
		return NULL;
	}

	// Set up data
	outImage = new char[imageSize.getNumberBytes()];

	// Read the ground truth data
	if ( readData( file_in, outImage, imageSize) )
	{
		cout << "loadRawDataImage: input file's image data not correct \n";
		delete [] outImage;
		file_in.close();
		return NULL;
	}

	file_in.close();
	return outImage;
}

bool TvRawImageIO::loadRawDataImage(char *file_in_name, 
									TvImageSize & imageSize, 
									char * imageData )
{
	// Set up file
	ifstream file_in;
	file_in.open ( file_in_name, std::ios::binary );
	
	if ( !file_in )
	{
		cout << "loadRawDataImage:  Cannot open the input file: " << file_in_name << "\n";
		return true;
	}

	// Load data from header
	TvImageSize tempImageSize;
	if ( loadRawDataFileHeader(file_in, tempImageSize) )
	{
		file_in.close();
		return true;
	}

	// Load the image if headers match
	if (tempImageSize == imageSize)
	{
	// Read the ground truth data
		if ( readData( file_in, imageData, imageSize) )
		{
			cout << "loadRawDataImage: input file's image data not correct \n";
			file_in.close();
			return true;
		}
		else
		{
			file_in.close();
			return false;
		}
	}
	else
	{
		cout << "loadRawDataImage: input image does not match input file \n";
		file_in.close();
		return true;
	}
}

//////////////////////////////////////////////////////////////////////////////
// Load Raw Data Methods
//////////////////////////////////////////////////////////////////////////////


bool TvRawImageIO::writeRawDataImage(
									 char *file_out_name, 
									 TvImageSize & imageSize, 
									 char * imageData, 
									 char * comments)
{
	ofstream file_out;
	file_out.open ( file_out_name, fstream::binary | fstream::out );

	if ( !file_out )
	{
		cout << "writeRawData:  Cannot open the file " << file_out_name << "\n";
		return true;
	}

	// Write header
	writeRawDataHeader(file_out, imageSize, comments);

	// Write the ground truth data
	if ( writeData( file_out, imageSize, imageData) )
	{
		cout << "writeRawData: output file failed to write \n";
		file_out.close();
		return true;
	}

	file_out.close();
	return false;
}

//////////////////////////////////////////////////////////////////////////////
// General Methods
//////////////////////////////////////////////////////////////////////////////

bool TvRawImageIO::byteSwapImage(
								 char * src, 
								 TvImageSize & imageSize
								 )
{ 
	if ( imageSize.dataSize == sizeof(short) )
	{	
		// Reference first byte in image
		char* byte1  = src;
		char* byte2  = src;
		byte2++;

		for ( int y=0; 
			y < (int)imageSize.getNumberElements(); 
			y++, byte1+=sizeof(short), byte2+=sizeof(short))
		{
			byteSwap(byte1,byte2);
		}
		return false;
	}
	else if ( imageSize.dataSize == sizeof(float) )
	{	
		// Reference first byte in image
		char* byte1  = src;
		char* byte2  = byte1 + 1;
		char* byte3  = byte2 + 1;
		char* byte4  = byte3 + 1;


		for ( int y=0; 
			y < (int)imageSize.getNumberElements(); 
			y++, 
			byte1+=sizeof(float), byte2+=sizeof(float),
			byte3+=sizeof(float), byte4+=sizeof(float)
			)
		{
			byteSwap(byte1,byte4);
			byteSwap(byte2,byte3);
		}
		return false;
	}
	else if ( imageSize.dataSize == sizeof(double) )
	{	
		// Reference first byte in image
		char* byte1  = src;
		char* byte2  = byte1 + 1;
		char* byte3  = byte2 + 1;
		char* byte4  = byte3 + 1;
		char* byte5  = src;
		char* byte6  = byte1 + 1;
		char* byte7  = byte2 + 1;
		char* byte8  = byte3 + 1;

		for ( int y=0; 
			y < (int)imageSize.getNumberElements(); 
			y++, 
			byte1+=sizeof(double), byte2+=sizeof(double),
			byte3+=sizeof(double), byte4+=sizeof(double),
			byte5+=sizeof(double), byte6+=sizeof(double),
			byte7+=sizeof(double), byte8+=sizeof(double)
			)
		{
			byteSwap(byte1,byte8);
			byteSwap(byte2,byte7);
			byteSwap(byte3,byte6);
			byteSwap(byte4,byte5);
		}
		return false;
	}
	else
	{
		cout << "ByteSwap: Data type not supported" << std::endl;
		return true;
	}
}


//////////////////////////////////////////////////////////////////////////////
// Internal Raw Data Methods
//////////////////////////////////////////////////////////////////////////////


void TvRawImageIO::writeRawDataHeader(
	ofstream &file_out, 
	TvImageSize & imageSize, 
	char * comments )
{
	if (comments)
		writeComments(file_out,comments);
	file_out << "# Width Height DataType(";
	file_out << writeDataType(imageSize) << ")\n";
	file_out << imageSize.width << " " 
		<< imageSize.height << " " 
		<< imageSize.dataType << "\n";
}

bool TvRawImageIO::loadRawDataFileHeader(ifstream & file_in, TvImageSize & imageSize )
{
	int width, height, dataType;
	if( readRawDataHeader(file_in, width, height, dataType) )
	{
		cout << "loadRawData:  Cannot read header of input file\n";
		return true;
	}

	// Check datatype max depth
	const char * temp = writeDataType(dataType);
	if (temp == NULL)
	{
		cout << "loadRawData: datatype not supported \n";
		return true;
	}

	imageSize.setImage(width,height,dataType);
	return false;
}

// Read the Raw Data Header
bool TvRawImageIO::readRawDataHeader(
	ifstream & file_in, 
	int & xsize, 
	int & ysize,
	int & dataType
	)
{
	// Set up reading parameters
	int count;
	char line[INTERNAL_STRINGSIZE];
	char *next;
	int step = 1;;
	int width;
	//char word[INTERNAL_STRINGSIZE];

	while ( true )
	{
		file_in.getline ( line, sizeof ( line ) );

		if ( file_in.eof() )
			return true;

		next = line;

		// Ignore comments
		if ( line[0] == '#' )
		{
			continue;
		}

		// Get width of image
		if ( step == 1 )
		{

			count = sscanf ( next, "%d%n", &xsize, &width );
			next = next + width;
			if ( count == EOF )
			{
				continue;
			}
			step = 2;
		}

		// Get height of image
		if ( step == 2 )
		{
			count = sscanf ( next, "%d%n", &ysize, &width );
			next = next + width;
			if ( count == EOF )
			{
				continue;
			}
			step = 3;
		}

		// Get size of data
		if ( step == 3 )
		{
			count = sscanf ( next, "%d%n", &dataType, &width );
			next = next + width;
			if ( count == EOF )
			{
				continue;
			}
			break;
		}
	}

	return false;

}


//////////////////////////////////////////////////////////////////////////////
// Internal PGM Methods
//////////////////////////////////////////////////////////////////////////////


void TvRawImageIO::writePgmHeader(
	ofstream &file_out, 
	TvImageSize & imageSize,
	const int maxValue)
{
	writeHeader(file_out, "P5");
	file_out << imageSize.width << " "
		<< imageSize.height << "\n"
		<< maxValue << "\n";
}


bool TvRawImageIO::readPgmHeader
(
	ifstream & file_in, 
	int & xsize, 
	int & ysize, 
	int & maxGreyVal,
	int nChannels
)
{
	// Set up reading parameters
	int count;
	char line[INTERNAL_STRINGSIZE];
	char *next;
	int step = 0;;
	int width;
	char word[INTERNAL_STRINGSIZE];

	while ( true )
	{
		file_in.getline ( line, sizeof ( line ) );

		if ( file_in.eof() )
			return true;

		next = line;

		if ( line[0] == '#' )
		{
			continue;
		}

		// Get magic identifier for binary PGM
		if ( step == 0 )
		{
			count = sscanf ( next, "%s%n", word, &width );
			if ( count == EOF )
			{
				continue;
			}
			next = next + width;
			if ( strcmp ( word, "P5" ) == 0 || strcmp ( word, "p5" ) == 0 )
			{
				if (nChannels != 1)	return true;
			}
			else if ( strcmp ( word, "P6" ) == 0 || strcmp ( word, "p6" ) == 0 )
			{
				if (nChannels != 3)	return true;
			}
			else
				return true;
			step = 1;
		}

		// Get width of image
		if ( step == 1 )
		{

			count = sscanf ( next, "%d%n", &xsize, &width );
			next = next + width;
			if ( count == EOF )
			{
				continue;
			}
			step = 2;
		}

		// Get height of image
		if ( step == 2 )
		{
			count = sscanf ( next, "%d%n", &ysize, &width );
			next = next + width;
			if ( count == EOF )
			{
				continue;
			}
			step = 3;
		}

		// Get max grey value in image
		if ( step == 3 )
		{
			count = sscanf ( next, "%d%n", &maxGreyVal, &width );
			next = next + width;
			if ( count == EOF )
			{
				continue;
			}
			break;
		}
	}

	return false;

}


bool TvRawImageIO::loadPGMfileHeader(ifstream & file_in, TvImageSize & imageSize)
{
	int width, height, maxGrey;
	if( readPgmHeader(file_in, width, height, maxGrey) )
	{
		cout << "loadPGM:  Cannot read header of input file\n";
		return true;
	}
	
	// Check max depth
	int depth;

	if (maxGrey <= MAX_8U)
		depth = TV_DATATYPE_8U;
	else if (maxGrey <= MAX_16U)
		depth = TV_DATATYPE_16U;
	else
	{
		cout << "loadPGM: Max depth of " << maxGrey << " not supported \n";
		return true;
	}

	imageSize.setImage(width,height,depth);
	return false;
}

bool TvRawImageIO::loadPPMfileHeader(ifstream & file_in, TvImageSize & imageSize)
{
	int width, height, maxGrey;
	const int nChannels = 3;
	if( readPgmHeader(file_in, width, height, maxGrey, nChannels) )
	{
		cout << "loadPPM:  Cannot read header of input file\n";
		return true;
	}
	
	// Check max depth
	int depth;

	if (maxGrey <= MAX_8U)
		depth = TV_DATATYPE_8U;
	else if (maxGrey <= MAX_16U)
		depth = TV_DATATYPE_16U;
	else
	{
		cout << "loadPPM: Max depth of " << maxGrey << " not supported \n";
		return true;
	}

	imageSize.setImage(width,height,depth, nChannels);
	return false;
}

bool TvRawImageIO::loadPGMdata(ifstream & file_in,
							   TvImageSize & imageSize,
							   char * imageData,
							   const bool bigEndian)
{
	// Read the PGM data
	if ( readData( file_in, imageData, imageSize) )
	{
		cout << "loadPGM: input file's image data not correct \n";
		return true;
	}

	if (imageSize.dataType == TV_DATATYPE_16U && bigEndian)
		byteSwapImage(imageData,imageSize);

	return false;
}

//////////////////////////////////////////////////////////////////////////////
// Internal General Methods
//////////////////////////////////////////////////////////////////////////////


// Read 2D data from file
bool TvRawImageIO::readData 
( ifstream &file_in, char * src, TvImageSize & imageSize)
{
	// Read data
	file_in.read( src, imageSize.getNumberBytes() );

	// Check for errors
	if( file_in.eof() )
	{
		cout << "PGMread: Error reading data" << std::endl;
		return true;
	}
	else
		return false;
}


// Write 2D data to file
bool TvRawImageIO::writeData( ofstream &file_out, TvImageSize & imageSize, char * src)
{

	// Write data
	file_out.write( src, imageSize.getNumberBytes() );

	// Check for errors
	if( file_out.eof() )
	{
		cout << "Error writing data" << std::endl;
		return true;
	}
	else
		return false;
}

bool TvRawImageIO::writeData( ofstream &file_out, TvImageSize & imageSize, char * src, bool byteSwap)
{
	// Check to see if byte swapping needed
	if(!byteSwap)
		return writeData(file_out,imageSize,src);
	else
	{
		char * tempC1 = src;
		char * tempC2 = src + 1;

		for (int y = 0 ; y < (int)imageSize.getNumberElements(); y++, tempC1+=2,tempC2+=2)
		{
			file_out.write( tempC2, sizeof(char) );
			file_out.write( tempC1, sizeof(char) );
		}	
		// Check for errors
		if( file_out.eof() )
		{
			cout << "Error writing data" << std::endl;
			return true;
		}
		else
			return false;

	}
}


const char * TvRawImageIO::writeDataType(int dataType)
{
	if(dataType == TV_DATATYPE_8U)
		return typeid(uchar).name();
	else if(dataType == TV_DATATYPE_8S)
		return typeid(char).name();
	else if(dataType == TV_DATATYPE_16U)
		return typeid(ushort).name();
	else if(dataType == TV_DATATYPE_16S)
		return typeid(short).name();
	else if(dataType == TV_DATATYPE_32F)
		return typeid(float).name();
	else if(dataType == TV_DATATYPE_64F)
		return typeid(double).name();
	else
		return NULL;

}