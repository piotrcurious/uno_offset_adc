
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <cstdlib>

using namespace std;

int main(int argc, char* argv[]) {
    if (argc < 2) {
        cerr << "Usage: test_ino <ino_file>" << endl;
        return 1;
    }
    string ino_file = argv[1];
    string test_cpp = "test_" + ino_file + ".cpp";

    ofstream out(test_cpp);
    out << "#include \"mock/Arduino.h\"" << endl;
    out << "#include \"mock/simulator.cpp\"" << endl;
    out << "#include \"" << ino_file << "\"" << endl;
    out.close();

    string compile_cmd = "g++ -o test_runner " + test_cpp + " -Imock -I.";
    int res = system(compile_cmd.c_str());
    if (res != 0) {
        cerr << "Compilation failed for " << ino_file << endl;
        return 1;
    }

    cout << "Running test for " << ino_file << "..." << endl;
    system("./test_runner");

    return 0;
}
