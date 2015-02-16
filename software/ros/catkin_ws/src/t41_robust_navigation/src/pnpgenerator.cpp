#include "pnpgenerator.h"

int node_id=0;  // unique id of the nodes
int arc_id=0;   // unique id of the arcs


static string place_color_str =
    "  <toolspecific tool=\"JARP\" version=\"1.2\">\n"
    "    <FrameColor>\n"
    "      <value>java.awt.Color[r=0,g=0,b=0]</value>\n"
    "    </FrameColor>\n"
    "    <FillColor>\n"
    "      <value>java.awt.Color[r=255,g=255,b=255]</value>\n"
    "    </FillColor>\n"
    "  </toolspecific>\n";


static string transition_color_str =
    "  <toolspecific tool=\"JARP\" version=\"1.2\">\n"
    "    <FrameColor>\n"
    "      <value>java.awt.Color[r=0,g=0,b=0]</value>\n"
    "    </FrameColor>\n"
    "    <FillColor>\n"
    "      <value>java.awt.Color[r=200,g=200,b=220]</value>\n"
    "    </FillColor>\n"
    "  </toolspecific>\n";

static string arc_color_str =
    "  <toolspecific tool=\"JARP\" version=\"1.2\">\n"
    "    <FrameColor>\n"
    "      <value>java.awt.Color[r=128,g=128,b=128]</value>\n"
    "    </FrameColor>\n"
    "    <ArrowMode>\n"
    "      <value>2</value>\n"
    "    </ArrowMode>\n"
    "  </toolspecific>\n";


std::ostream& operator<< (std::ostream& stream, const Place& place) {
    stream << "<place id=\"" << place.sid << "\">\n"
            << "  <graphics>\n"
            << "    <position x=\"" << place.posx << "\" y=\"" << place.posy << "\" />"
            << "    <size width=\"32\" height=\"32\" />"
            << "  </graphics>\n"
            << "  <name>\n"
            << "    <value>" << place.name << "</value>\n"
            << "    <graphics>\n"
            << "      <offset x=\"0\" y=\"40\" />\n"
            << "    </graphics>\n"
            << "  </name>\n"
            << "  <initialMarking>\n"
            << "    <value>" << place.marking << "</value>\n"
            << "  </initialMarking>\n"
            << place_color_str
            << "</place>\n";
    return stream;
}



std::ostream& operator<< (std::ostream& stream, const Transition& transition) {
    stream << "<transition id=\"" << transition.sid << "\">\n"
            << "  <graphics>\n"
            << "    <position x=\"" << transition.posx << "\" y=\"" << transition.posy << "\" />"
            << "    <size width=\"8\" height=\"32\" />"
            << "  </graphics>\n"
            << "  <name>\n"
            << "    <value>" << transition.name << "</value>\n"
            << "    <graphics>\n"
            << "      <offset x=\"0\" y=\"-20\" />\n"
            << "    </graphics>\n"
            << "  </name>\n"
            << transition_color_str
            << "</transition>\n";
    return stream;
}


std::ostream& operator<< (std::ostream& stream, const Arc& arc) {
    stream << "<arc id=\"" << arc.sid << "\" source=\"" << arc.source << "\" target=\"" << arc.target << "\" >\n"
            << "  <inscription>\n"
            << "    <value>1</value>\n"
            << "    <graphics>\n"
            << "      <offset x=\"0\" y=\"40\" />\n"
            << "    </graphics>\n"
            << "  </inscription>\n"
            << arc_color_str
            << "</arc>\n";
    return stream;
}

std::ostream& operator<< (std::ostream& stream, const Edge& edge) {
    stream << edge.a;
    return stream;
}


std::ostream& operator<< (std::ostream& stream, const PNP& pnp) {

    stream << "<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>\n\n"
            << "<pnml>\n"
            << "  <!--generator: PNPgenerator (Luca Iocchi 2015)-->\n"
            << "  <net id=\"n1\" type=\"PTNet\">\n"
            << "    <name>\n"
            << "      <value>" << pnp.name << "</value>\n"
            << "    </name>\n";

    vector<Place*>::const_iterator ip = pnp.P.begin();
    while (ip!=pnp.P.end()) {
        Place *p = *ip;
        stream << *p; ip++;
    }
    vector<Transition*>::const_iterator it = pnp.T.begin();
    while (it!=pnp.T.end()) {
        Transition *t = *it;
        stream << *t; it++;
    }
    vector<Edge*>::const_iterator ie = pnp.E.begin();
    while (ie!=pnp.E.end()) {
        Edge *e = *ie;
        stream << *e; ie++;
    }

    stream << "  </net>\n"
            << "</pnml>\n";
    return stream;
}



Place* PNP::addPlace(string name) {
    Place* p = new Place(name);
    P.push_back(p);
    return p;
}


Transition* PNP::addTransition(string name) {
    Transition* t = new Transition(name);
    T.push_back(t);
    return t;
}


void PNP::connect(Node* n1, Node* n2) {
    Edge *e = new Edge(n1,n2);
    E.push_back(e);
}

Place* PNP::addCondition(string name, Place* p0) {
    Transition *t = addTransition(name);
    Place *p1 = addPlace("X");
    connect(p0,t); connect(t,p1);
    return p1;
}

Place* PNP::addAction(string name, Place* p0) {
    Transition *ts = addTransition(name+".start");
    Place *pe = addPlace(name+".exec");
    Transition *te = addTransition(name+".end");
    Place *pf = addPlace("X");
    connect(p0,ts); connect(ts,pe); connect(pe,te); connect(te,pf);
    return pf;
}
