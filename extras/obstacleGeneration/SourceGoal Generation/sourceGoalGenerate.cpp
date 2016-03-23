#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <time.h>

using namespace std;

struct Position
{
float x, y;
}startPos,endPos;

struct Obstacle
{
Position a, b, c;
};

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

struct Obstacle getObstacleFromValue(float x, float y, float height, float width)
{
    Obstacle obstacle;
    Position A;
    A.x = x;
    A.y = y;

    Position B;
    B.x = x;
    B.y = y + height;

    Position C;
    C.x = x + width;
    C.y = y;

    obstacle.a = A;
    obstacle.b = B;
    obstacle.c = C;

    return obstacle;
}


vector<Obstacle> getObstaclesFromFile(char * filename)
{
    Obstacle obstacle;
    vector<Obstacle> myObstacles;
    string line;
    ifstream myfile (filename);
    if (myfile.is_open())
    {
        float x,y,height,width;
        while ( getline (myfile,line) )
        {
            vector<string> temp = split(line,',');
            x = atof(temp[0].c_str());
            y = atof(temp[1].c_str());
            height = atof(temp[3].c_str());
            width = atof(temp[2].c_str());
            obstacle = getObstacleFromValue(x,y,height,width);
            myObstacles.push_back(obstacle);
        }
        myfile.close();
    }
    else cout << "Unable to open file";
    return myObstacles;
}




bool checkInsideObstacles(vector< Obstacle > &obstArray, Position point)
{
    double AB, AD, AMAB, AMAD;

    for(int i=0; i<obstArray.size(); i++)
    {
        AB = (pow(obstArray[i].a.x - obstArray[i].b.x,2) + pow(obstArray[i].a.y - obstArray[i].b.y,2));
        AD = (pow(obstArray[i].a.x - obstArray[i].c.x,2) + pow(obstArray[i].a.y - obstArray[i].c.y,2));
        AMAB = (((point.x - obstArray[i].a.x) * (obstArray[i].b.x - obstArray[i].a.x)) + ((point.y - obstArray[i].a.y) * (obstArray[i].b.y - obstArray[i].a.y)));
        AMAD = (((point.x - obstArray[i].a.x) * (obstArray[i].c.x - obstArray[i].a.x)) + ((point.y - obstArray[i].a.y) * (obstArray[i].c.y - obstArray[i].a.y)));
         //(0<AM⋅AB<AB⋅AB)∧(0<AM⋅AD<AD⋅AD)
        if((0 < AMAB) && (AMAB < AB) && (0 < AMAD) && (AMAD < AD))
        {
            return true;
        }
    }
    return false;
}


void writeTofile(char* filename)
{
    ofstream outFile;
    outFile.open(filename,ios_base::out);
    outFile << startPos.x << "," << startPos.y << endl;
    outFile << endPos.x << "," << endPos.y << endl;
    outFile.flush();
    outFile.close();
    //cout<< "file written";
}

Position getNewRandomStartPosition()
{
    Position pos;
    pos.x = rand() % 10;
    pos.y = rand() % 100;
    return pos;
}

Position getNewRandomEndPosition()
{
    Position pos;
    pos.x = (rand() % 10) + 90;
    pos.y = (rand() % 10);
    return pos;
}

int main(int passedArgumentCount,char** passedArgumentValues)
{
    if(passedArgumentCount != 3)// Check the value of passedArgumentCount. if filename is not passed
    {
        cout<<"count not proper";
        exit(0);
    }
    if(passedArgumentValues[1] == '\0')
    {
        cout<<"IN Filename not proper";
        return -1;
    }

    if(passedArgumentValues[2] == '\0')
    {
        cout<<"Out Filename not proper";
        return -1;
    }

    srand(time(NULL));

    vector<Obstacle> obstacleArray = getObstaclesFromFile(passedArgumentValues[1]);

    while(true)
    {
        startPos = getNewRandomStartPosition();
        if(!checkInsideObstacles(obstacleArray,startPos))
            break;
    }

    while(true)
    {
        endPos = getNewRandomEndPosition();
        if(!checkInsideObstacles(obstacleArray,endPos))
            break;
    }

    writeTofile(passedArgumentValues[2]);
    return 1;
}

