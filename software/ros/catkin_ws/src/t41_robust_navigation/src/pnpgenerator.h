#ifndef PNPGENERATOR_H
#define PNPGENERATOR_H

#include <string>
#include <sstream>
#include <vector>

using std::string;
using std::stringstream;
using std::vector;

static int dx=60, dy=60, offx=20, offy=200;  // draw parameters

extern int node_id;  // unique id of the nodes
extern int arc_id;   // unique id of the arcs


class Node {
protected:


    int X,Y; // virtual position
    int posx,posy; // real position

    string name; // name of the node
    string sid; // string version of the id

public:
    Node(string _name) : name(_name) {
        stringstream ss; ss << "n" << node_id; sid = ss.str();
        X=node_id; Y=0; setX(X); setY(Y); node_id++;
    }

    int getX() {
       return X;
    }

    void setX(int x) {
        X=x; posx = offx + X * dx;
    }

    int getY() {
        return Y;
    }

    void setY(int y) {
        Y=y; posy = offy + Y * dy;
    }

    string getID() {
        return sid;
    }

    void setName(string _name) {
        name=_name;
    }
};


class Place : public Node {
    int marking;
public:
    Place(string _name) : Node(_name), marking(0) { }

    void setInitialMarking() {
        marking=1;
    }

    friend std::ostream& operator<< (std::ostream& stream, const Place& place);

};

class Transition : public Node {
public:
    Transition(string _name) : Node(_name) { }

    friend std::ostream& operator<< (std::ostream& stream, const Transition& transition);
};


class Arc {

    string sid, source, target;
public:
    Arc(string _source, string _target) {
        stringstream ss; ss << "a" << arc_id; sid = ss.str(); arc_id++;
        source=_source; target=_target;
    }

    string getID() {
        return sid;
    }

    friend std::ostream& operator<< (std::ostream& stream, const Arc& arc);
};

class Edge {
    Node *n1, *n2;
    Arc a;
public:
    Edge(Node* _n1, Node* _n2) : n1(_n1), n2(_n2), a(n1->getID(),n2->getID()) {
    }

    friend std::ostream& operator<< (std::ostream& stream, const Edge& edge);
};

class PNP {

protected:
    string name;
    vector<Place*> P;
    vector<Transition*> T;
    vector<Edge*> E;

public:
    PNP(string _name) : name(_name) {
        node_id=0;  arc_id=0;
    }

    Place* addPlace(string name);
    Transition* addTransition(string name);
    void connect(Node* n1, Node* n2);
    Place* addCondition(string name, Place* p0);
    Place* addAction(string name, Place* p0);

    friend std::ostream& operator<< (std::ostream& stream, const PNP& pnp);
};

class PNPGenerator
{
private:


public:
    PNPGenerator();
};

#endif // PNPGENERATOR_H
