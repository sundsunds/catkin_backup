// This file was written by Toby Vaudrey
// Published on http://www.cs.auckland.ac.nz/~tvau003/CodeExamples/tvImageIO.zip

#ifndef FILE_tvRawImageIO_h
#define FILE_tvRawImageIO_h

#include <fstream>

typedef unsigned char uchar;
typedef unsigned short ushort;

#define TV_DATATYPE_8U     8	// 8-bit unsigned char
#define TV_DATATYPE_8S    -8   // 8-bit signed char
#define TV_DATATYPE_16U   16	// 16-bit unsigned char (unsigned short)
#define TV_DATATYPE_16S  -16	// 16-bit unsigned char (short)
#define TV_DATATYPE_32F   32	// 32-bit float (float)
#define TV_DATATYPE_64F   64	// 64-bit float (double)
#define MAX_8U 255
#define MAX_16U 65535
#define INTERNAL_STRINGSIZE 255


// Structure for defining size of an image
struct TvImageSize
{
	// Definition
	TvImageSize(int f_width = 0, int f_height = 0, int f_dataType = 0, int f_nChannels = 1)
	{ setImage(f_width, f_height, f_dataType, f_nChannels); };
	~TvImageSize(){;};

	// Equal to operator
	bool operator== (const TvImageSize& imageSize) const
	{
		return (
			imageSize.width == width && 
			imageSize.height == height &&
			imageSize.dataType == dataType &&
			imageSize.dataSize == dataSize );
	};

	TvImageSize operator= (const TvImageSize& imageSize) const
	{
		TvImageSize outSize;
		outSize.width = imageSize.width;
		outSize.height = imageSize.height;
		outSize.dataSize = imageSize.dataSize;
		outSize.dataType = imageSize.dataType;
		return outSize;
	};

	// Resets the image data
	void setImage(int f_width, int f_height, int f_dataType, int f_nChannels = 1)
	{
		width = f_width;
		height = f_height;
		dataType = f_dataType;
		nChannels = f_nChannels;
		setDataSize();
	};

	// Set the data size of individual values
	void setDataSize()
	{
		if( dataType == TV_DATATYPE_8U || dataType == TV_DATATYPE_8S)
			dataSize = sizeof(char);
		else if( dataType == TV_DATATYPE_16U || dataType == TV_DATATYPE_16S)
			dataSize = sizeof(short);
		else if( dataType == TV_DATATYPE_32F )
			dataSize = sizeof(float);
		else if( dataType == TV_DATATYPE_64F )
			dataSize = sizeof(double);
		else
			dataSize = 0;
	};

	// Return number of elements / pixels in an image
	unsigned int getNumberElements()
	{		return width*height*nChannels;	};

	// Return the number of bytes in the image
	unsigned int getNumberBytes()
	{		return width*height*dataSize*nChannels;	};

	// Internal Data
	int width;
	int height;
	int dataSize;
	int dataType;
	int nChannels;
};

using namespace std;

// Image reading/writing class
class TvRawImageIO
{

public:
	TvRawImageIO() {;} ;
	~TvRawImageIO() {;} ;

	// Load PGM to array of (unsigned) chars
	// Inputs: 
	// file_in_name = file to load
	// bigEndian = if 16-bit image, whether the input file is big
	//			   endian or not.
	// src = pointer to pre-loaded image, i.e. use this image to store data.
	// Input/Outputs
	// imageSize = if no input image given then sets this to have image
	//             information. Otherwise, uses this for data
	char * loadPGM(
		char *file_in_name, 
		TvImageSize & imageSize,
		const bool bigEndian = true );
	
	bool loadPGM(
		char *file_in_name, 
		TvImageSize & imageSize,
		char * imageData,
		const bool bigEndian = true );
	
	// Write PGM to file
	// Inputs: 
	// file_out_name = file to load
	// bigEndian = if 16-bit image, whether to output as BigEndian image or not
	// src = pointer to image data
	// imageSize = used to write data to file
	// Outputs:
	// Returns 1 if there was an error, 0 otherwise
	bool writePGM(char *file_out_name, TvImageSize & imageSize, char * src, const bool bigEndian = true );


	// Load PPM to array of (unsigned) chars
	// Inputs: 
	// file_in_name = file to load
	// bigEndian = if 16-bit image, whether the input file is big
	//			   endian or little endian.
	// src = pointer to pre-loaded image, i.e. use this image to store data.
	// Input/Outputs
	// imageSize = if no input image given then sets this to have image
	//             information. Otherwise, uses this for data
	// Outputs, a striped array containing RGB 
	char * loadPPM(
		char *file_in_name, 
		TvImageSize & imageSize,
		const bool bigEndian = true );
	

	// Raw image format functions
	// Used for reading/writing float, double, char, uchar, short, and ushort 2D data to file
	char * loadRawDataImage(char *file_in_name, TvImageSize & imageSize);
	bool loadRawDataImage(char *file_in_name, TvImageSize & imageSize, char * imageData);
	bool writeRawDataImage(char *file_out_name, TvImageSize & imageSize, char * imageData, char * comments = NULL);

	bool byteSwapImage(char * src, TvImageSize & imageSize);

private:

	// PGM Functions
	bool readPgmHeader(ifstream & file_in, int & xsize, int & ysize, int & maxGreyVal, 
		// Filetype
		// PGM = 1
		// PPM = 3
		int nChannels = 1 );
	void writePgmHeader(ofstream &file_out, TvImageSize & imageSize, const int maxValue);
	bool loadPGMfileHeader(ifstream & file_in, TvImageSize & imageSize );
	bool loadPPMfileHeader(ifstream & file_in, TvImageSize & imageSize );
	bool loadPGMdata(ifstream & file_in, TvImageSize & imageSize, char * imageData, const bool bigEndian);

	inline void byteSwap(char * byte1, char * byte2)
	{
		// Store 1st byte value
		char tempChar = *byte1;
		// Swap bytes
		*byte1 = *byte2;
		*byte2 = tempChar;
	}

	// Raw Data file functions
	bool readRawDataHeader(ifstream & file_in, int & xsize, int & ysize, int & dataType);
	void writeRawDataHeader(ofstream &file_out, TvImageSize & imageSize, char * comments = NULL);
	bool loadRawDataFileHeader(ifstream & file_in, TvImageSize & imageSize );

	// Generic data functions
	bool readData( ifstream &file_in, char * src, TvImageSize & imageSize);
	bool writeData ( ofstream &file_out, TvImageSize & imageSize, char * src);
	bool writeData ( ofstream &file_out, TvImageSize & imageSize, char * src, bool byteSwap);

	void writeHeader(ofstream &file_out, const char * header)
	{
		if (header)
			file_out << header << "\n";
	}
	void writeComments( ofstream &file_out, const char * comments )
	{
		if (comments) 
			file_out << "# " << comments << "\n";
	}
	const char * writeDataType(int dataType);
	const char * writeDataType(TvImageSize & imageSize)
	{ return writeDataType(imageSize.dataType);		};
};



#endif // FILE_tvRawImageIO_h