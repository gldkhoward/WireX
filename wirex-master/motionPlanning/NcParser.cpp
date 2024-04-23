/*
* WireX  -  motionPlanning
*
* Copyright (c) 2006-2019 Andreas Pott
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

/*!*******************************************************************
 *  \file   : NcParser.cpp
 *
 *  Project   motionPlanning
 *
 *  \Author   Werner Kraus
 *
 *********************************************************************
 */ 

#include <map>
#include "NcParser.h"
#include <iostream>
#include <fstream>
#include <sstream>


namespace PCRL {

CNcParser::CNcParser(void)
{
	eGCommand=NC_SAVE;
	eMCommand=NC_PRINT;
	eSharpCommand=NC_PRINT;
	eComment=NC_IGNORE;

	mAxis["W1"]=1;
	mAxis["W2"]=2;
	mAxis["W3"]=3;
	mAxis["W4"]=4;
	mAxis["W5"]=5;
	mAxis["W6"]=6;

}

CNcParser::~CNcParser(void)
{
	;
}

void CNcParser::setAxisMap(int iAxis, string cName)
{
	// futher implementation to define the connection between the axis defined in G-Code and 
	/*
	map<CString,int>::iterator iter;
	for (iter=mAxis.begin(); iter != mAxis.end(); ++iter) {
		if (iter->second==iAxis)
		{
			//iter->first=&cName;
			break;
		}
    }
*/
}



void CNcParser::setHandling(handling eGCommand, handling eMCommand, handling eSharpCommand,handling eComment)
{
	this->eGCommand=eGCommand;
	this->eMCommand=eMCommand;
	this->eSharpCommand=eSharpCommand;
	this->eComment=eComment;

}



/*! Open a Nc-Programm and interprete it.
* The Nc-Commands are saved in Programm, which class has inherited the 
* functionality from std::list
 */
bool CNcParser::ParseNcfile(const char* filepath, CNcProgram& Program)
{
	typedef pair <int, double> Pair; // Pair for the map between the NC-Variables and their values

 	double W[6];
	double W_temp[6];
	double velocity=0;
	double wait_time=0;
	for (int i=0;i<6;i++)
	{
		W[i]=0;
		W_temp[i]=0;
	}
	int counter=0;

	std::fstream filestr;
	filestr.open (filepath , fstream::in | fstream::app);

	while(!filestr.eof()) // read in one line after another
	{
		counter++;
		char buf[2048];
		filestr.getline(buf,2048,'\n');
		string sInput=buf;

		if(sInput.find("(",0)!=string::npos) // handle with comments
		{
			int iStart=0, iEnd=0;
			iStart=sInput.find("(",iStart); // find the begin
			iEnd=sInput.find(")",iStart+1);	// find the end
			if (iEnd==string::npos)					// when there is no end, use the last sign
				iEnd=sInput.length()-1;
			sInput.erase(iStart,iEnd-iStart+1); // delete the comment
		}
		trim(sInput); // delete blanks at the begin and the end of the string

		int iStart=0, iEnd=0; 
		int iValue;
		int iAxis;
		double dValue1;
		bool bNewInstructionSet=false; // mark a new instruction set
		while (iEnd< (int)sInput.length())
		{
			switch ((unsigned char)sInput[iStart]) // analyse the first character
			{
				case 'G':
				{	
					if (readInteger(sInput, ++iStart, iEnd, iValue))
					{
						bNewInstructionSet=true;
						switch (iValue) // interprete the G-command
						{
						case 0:
						{
							eMotionMode=MOTION_MODE_SEEK;
							break;
						}
						case 1:
						{
							eMotionMode=MOTION_MODE_LINEAR;
							break;
						}
						case 2:
						{
							eMotionMode=MOTION_MODE_CW_ARC;
							break;
						}
						case 3:
						{
							eMotionMode=MOTION_MODE_CCW_ARC;
							break;
						}
						case 4:
						{
							// wait
							readValue(sInput, ++iStart, iEnd, wait_time);
							eMotionMode=MOTION_MODE_DWELL;
							cout << "Wartezeit: " << wait_time << endl;
							break;
						}
						case 90:
						{
							ePositionMode=POSITION_ABSOLUTE;
							for (int i=0;i<6;i++)
								W_temp[i]=W[i]; // use stored values for absolute movement
							break;
						}
						case 91:
						{
							ePositionMode=POSITION_RELATIVE;
							for (int i=0;i<6;i++)
								W_temp[i]=0; // reset the stored values for relative movement
							break;
						}
						default:
							cout << "unknown G-Command: " << iValue << endl;
					}

						
					}
					break;
				}
				case 'N':  // N indicates the line number
				{	
					readInteger(sInput, ++iStart, iEnd, iValue);
					break;
				}
				case '=':
				{	
					readValue(sInput, iStart, iEnd,dValue1);
					W_temp[iAxis-1]=dValue1; // write the value into the temp array
					break;
				}
				case '+': // handle with additions
				{
					if ((unsigned char)sInput[iStart+1]=='P') // the value after + is defined by a variable
					{	
						iStart++;
						readInteger(sInput, ++iStart, iEnd, iValue);
						cout << "addition " << iValue << endl;
						dValue1=mVariables.find(iValue)->second;
						//break;
					}
					else
					{
						readDouble(sInput, ++iStart,iEnd,dValue1);
					}
					W_temp[iAxis-1]+=dValue1;
					break;
				}
				case'-': // handle with subtractions
				{
					readValue(sInput, iStart, iEnd,dValue1);
					W_temp[iAxis-1]-=dValue1;
					break;
				}

				case '#': // special commands, not interpreted yet
				{
					iEnd= (int)sInput.length();
					break;
				}

				case 'V': // special commands, not interpreted yet
				{
					iEnd= (int)sInput.length();
					break;
				}

				case '%': // special commands, not interpreted yet
				{
					iEnd= (int)sInput.length();
					break;
				}

				case '$': // special commands, not interpreted yet
				{
					iEnd= (int)sInput.length();
					break;
				}

				case'F': // F means the speed, typically in mm/min
				{
					readValue(sInput, iStart, iEnd,velocity);
					break;
				}

				case 'M': // Machine Commands
				{
					readInteger(sInput, ++iStart, iEnd, iValue);
					cout << "M-command No: " << iValue << endl;
					break;
				}

				case 'P': // initialization of variables
				{	// this case is reached, when the string looks "P12=12.123" in difference to "W1=P12"
					readInteger(sInput, ++iStart, iEnd, iValue); // get the variable nr
					iStart=iEnd;
					
					while ((unsigned char)sInput[iStart+1]==' ') // ignores spaces
						iStart++;
					if ((unsigned char)sInput[iStart+1]=='=')
					{
						iStart++;
						readDouble(sInput, ++iStart,iEnd,dValue1); // get the variable value
						mVariables.insert(Pair(iValue, dValue1));  // create a pair in the map
					}
					break;
				}

				case 'W': // W stands for the axis, e.g. "W1" for the x-axis
				{
					readInteger(sInput, ++iStart, iEnd, iAxis); // read the nr of the axis and save it in iAxis
					break;
				}

				case ' ': // ignore spaces
				{
					iEnd++;
					break;
				}
					
				default:
				{
					cout << "not interpreted:" << sInput[iStart]<< " | " << sInput << endl; // print out not interpreted characters
					iEnd++;
					iEnd= (int)sInput.length();
				}
			}
			iStart=iEnd;
		}

		if (bNewInstructionSet==true) // if there is a new instruction set, save the values stored in w_temp and push back to list programm
		{
			if (ePositionMode==POSITION_ABSOLUTE)
			{
				for (int i=0;i<6;i++)
				{
					W[i]=W_temp[i];
				}
			}
			else if (ePositionMode==POSITION_RELATIVE)
			{
				for (int i=0;i<6;i++)
				{
					W[i]+=W_temp[i];
					W_temp[i]=0; // reset the stored values for relative movement
				}
			}
			for (int i=0;i<6;i++) // only for information
			{
				;//cout << W[i]<< " ";
			}
			bNewInstructionSet=false;
			//cout << " " << velocity << endl;
			switch (eMotionMode)
			{
				case MOTION_MODE_SEEK:
				{
					break;
				}
				case MOTION_MODE_LINEAR:
				{
					CNcLinear* pLinear=new CNcLinear();
					pLinear->targetposition=Vector3d(W[0],W[1],W[2]);
					pLinear->targetorientation=Vector3d(W[3],W[4],W[5]);
					pLinear->v=velocity;
					Program.push_back(pLinear);
					break;
				}
				case MOTION_MODE_CW_ARC:
				{
					break;
				}
				case MOTION_MODE_CCW_ARC:
				{
					break;
				}
				case MOTION_MODE_DWELL:
				{
					CNcDwellTime* pDwellTime=new CNcDwellTime();
					pDwellTime->targetposition=Vector3d(W[0],W[1],W[2]);
					pDwellTime->targetorientation=Vector3d(W[3],W[4],W[5]);
					pDwellTime->time=wait_time;
					Program.push_back(pDwellTime);
					break;
				}

			}
		}

	}

	/*
	cout << "defined Variables and their values:" << endl;
	map <int, double>::iterator Iter1;

	for ( Iter1 = mVariables.begin( ); Iter1 != mVariables.end( ); Iter1++ )
	{
		cout << "P" << Iter1 -> first << "  ";
		cout << " " << Iter1 ->second;
		cout << "." << endl;
	}
	*/
	return true;
}


bool CNcParser::exportNcProgram(CNcProgram& Program)
{
	ofstream out;
	out.open ("NcProgram.txt");
	
	list<CNcCommand*>::const_iterator it_Nc=Program.begin(); // Iterator 1

	for (it_Nc;it_Nc!=Program.end();it_Nc++) 
	{
		CNcLinear* nc_lin=dynamic_cast<CNcLinear *>(*it_Nc); 
		
		CNcDwellTime* nc_dwell=dynamic_cast<CNcDwellTime *>(*it_Nc); 
		
		if (nc_lin!=0)
		{
			out << nc_lin->targetposition.x() << " ; " << nc_lin->targetposition.y() << " ; " << nc_lin->targetposition.z() << " ; " << nc_lin->targetorientation.x() << " ; " << nc_lin->targetorientation.y() << " ; " << nc_lin->targetorientation.z() << "\n";
		}

	}

	return true;
}


/*! help function to read a double value from a string beginning at position iStart
*   the position of the first letter, which is not numeric is returned in iEnd
 */
bool CNcParser::readDouble(string& sInput, int& iStart, int& iEnd, double& dValue)
{
	char *ep1;
	char *buffer = new char[sInput.substr(iStart).length()+1];
	strcpy(buffer,sInput.substr(iStart).c_str());
	dValue=strtod(buffer, &ep1);
	iEnd=sInput.length()-strlen(ep1);
	delete buffer;
	return iEnd>iStart;
}

/*! help function to read an integer value from a string beginning at position iStart
*   the position of the first letter, which is not numeric is returned in iEnd
 */
bool CNcParser::readInteger(string& sInput, int& iStart, int& iEnd, int& iValue)
{
	char *ep1;
	char *buffer = new char[sInput.substr(iStart).length()+1];
	strcpy(buffer,sInput.substr(iStart).c_str());
	iValue=(int)strtol(buffer, &ep1, 10);
	iEnd=sInput.length()-strlen(ep1);
	delete buffer;
	return iEnd>iStart;
}

/*! help function to read a value which can be defined by a double or a variable
 */
bool CNcParser::readValue(string& sInput, int& iStart, int& iEnd, double& dValue)
{
	int iValue;
	while ((unsigned char)sInput[iStart+1]==' ') // ignore spaces
		iStart++;
	if ((unsigned char)sInput[iStart+1]=='P') // the value after = is defined by a variable
	{	
		iStart++;
		readInteger(sInput, ++iStart, iEnd, iValue); // read the nr of the variable
		dValue=mVariables.find(iValue)->second; // get the corresponding value from the map
	}
	else
	{
		readDouble(sInput, ++iStart,iEnd,dValue); 
	}
	return true;
}

/*! help function to trim a string
 */
void CNcParser::trim(string& str)
{
	string::size_type pos = str.find_last_not_of(' ');
	if(pos != string::npos) 
	{
		str.erase(pos + 1);
		pos = str.find_first_not_of(' ');
		if(pos != string::npos) str.erase(0, pos);
	}
	else 
		str.erase(str.begin(), str.end());
}

} // end of namespace PCRL