#include <iostream>
#include "./message.h"


using namespace std;

message::message(){

}

message::~message(){

}

ostream& operator<<(ostream& os, const message& self){
    os << "px: " << self.px << " py: " << self.py << endl
        << " v: " << self.v << " a: " << self.a << endl
        << "psi: " << self.psi << " delta: " << self.delta << endl;

    return os;
}
