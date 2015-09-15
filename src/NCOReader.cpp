#include <path_planning/NCOReader.h>
#include <fstream>

/*
 * Split Function
 */
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

/*
 * Split Function
 */
std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

/*
 * Function to read obstacle points from a file
 */
void NCOReader::readNCO(char* filename)
{
    Point currentPoint;
    vector< Point > myObstacles;
    string currentLine;
    ifstream myFile (filename);
    if (myFile.is_open())
    {
        while ( getline (myFile, currentLine) )
        {
            vector<string> temp = split(currentLine,',');
            currentPoint.x = atof(temp[0].c_str());
            currentPoint.y = atof(temp[1].c_str());
            currentPoint.z = 0;
            myObstacles.push_back(currentPoint);
        }
        myFile.close();
    }
    else cout << "Unable to open file";
    this->obstaclePoints = myObstacles;
}


/*
 * Function to convert obstacle points to lines
 */
void NCOReader::convertToLines()
{
    NCOLine ncoLine;
    int j;
    for (int i = 0; i < this->obstaclePoints.size(); ++i)
    {
        ncoLine.start = this->obstaclePoints[i];

        //getting the next point for line
        j = (i+1)%this->obstaclePoints.size();

        ncoLine.end = this->obstaclePoints[j];

        this->obstacleLines.push_back(ncoLine);
    }
}



/*
 * constructor
 */
NCOReader::NCOReader(char* points, char* rects)
{
    readNCO(points);
    readNCORects(rects);
    convertToLines();
}

/*
 * Read obstacles in rectangle description for the passed obstacle
 */
void NCOReader::readNCORects(char *filename)
{
    vector < Point > rectangle;
    float x, y, width, height;
    Point point;
    string lineToRead;
    ifstream myFile (filename);
    if (myFile.is_open())
    {
        while ( getline (myFile,lineToRead) )
        {
            vector<string> temp = split(lineToRead,',');
            x = atof(temp[0].c_str());
            y = atof(temp[1].c_str());
            height = atof(temp[3].c_str());
            width = atof(temp[2].c_str());


            //first point
            point.x = x;
            point.y = y;
            point.z = 0;

            rectangle.push_back(point);

            //second point
            point.x = x;
            point.y = y + height;
            point.z = 0;

            rectangle.push_back(point);

            //third point
            point.x = x + width;
            point.y = y + height;
            point.z = 0;

            rectangle.push_back(point);

            //fourth point
            point.x = x + width;
            point.y = y;
            point.z = 0;
            rectangle.push_back(point);

            //first point again to complete the box
            point.x = x;
            point.y = y;
            point.z = 0;
            rectangle.push_back(point);

            this->obstacleRects.push_back(rectangle);
            rectangle.clear();
        }
        myFile.close();
    }
    else
    {
        cout << "Unable to open file";
        exit(-1);
    }
}



/*
 * function to set start and end point by reading it from a file
 */
void NCOReader::setSourceGoalPoints(char *filename, Point &start, Point &goal)
{
    ifstream inFile;
    inFile.open(filename);
    string line;
    if (inFile.is_open())
    {
        //read Source
        getline (inFile,line);
        vector<string> temp = split(line,',');
        start.x = atof(temp[0].c_str());
        start.y = atof(temp[1].c_str());

        //read Goal
        getline (inFile,line);
        temp = split(line,',');
        goal.x = atof(temp[0].c_str());
        goal.y = atof(temp[1].c_str());

        inFile.close();
    }
    else
    {
        cout << "Unable to open file";
    }
}

vector< Point > NCOReader::getObstaclePoints()
{
    return this->obstaclePoints;
}

vector< NCOLine > NCOReader::getObstacleLines()
{
    return this->obstacleLines;
}

vector< vector < Point > > NCOReader::getObstacleRects()
{
    return this->obstacleRects;
}