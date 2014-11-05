#ifndef CONFIG_H
#define CONFIG_H

#include <vector>
#include <iostream>

using namespace std;

class Position2D
{
    public:
        Position2D(float a, float b)
        {
            x=a; y=b;
        }

        friend ostream& operator<<(ostream& os, const Position2D& p)
        {
            os << "(" << p.x << ',' << p.y << ")";
            return os;
        }

        float x, y;

};


class Config
{
    public:
        Config(string name);

        bool hasEnded();

        // MAP_DIMENSIONS
        float width;
        float height;
        float cellWidth;
        float cellHeight;

        // MINES
        int numMines;
        bool randomMines;
        float detectionMinDist;
        float explosionMaxDist;
        vector<Position2D> minesPositions;

    private:
        string filename;
};

#endif /* CONFIG_H */
