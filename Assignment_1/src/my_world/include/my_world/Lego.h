#ifndef LEGO_H
#define LEGO_H

class Lego{
    private:
        std::string name;
        double x;
        double y;
        int classe;
    
    public:
        Lego();
        Lego(std::string _name, const double _x, const double _y, const int classe);
        void setName(std::string _name);
        void setX(const double _x);
        void setY(const double _y);
        void setClasse(const int _c);
        std::string getName();
        double getX();
        double getY();
        int getClasse();
        double getGrip();   //for each lego class there is a different gripper aperture
        double getZapproach();   //for each lego class there is a different altitude at wich it has to be picked up
};

#endif

