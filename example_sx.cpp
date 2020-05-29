/**
 * An example of OpenCX, especially for 'opensx.hpp'
 */

#include "opencx.hpp"
#include <iostream>

using namespace std;

int testStringOperations(const string& text = "\t  OpenCX makes OpenCV v4 easier!\n  ")
{
    cout << "### String Trimming" << endl;
    cout << "* Original text: " << text << endl;
    cout << "* Left-trimmed text: " << cx::trimLeft(text) << endl;
    cout << "* Right-trimmed text: " << cx::trimRight(text) << endl;
    cout << "* Both-trimmed text: " << cx::trimBoth(text) << endl;
    cout << endl;

    cout << "### String toLowerCase" << endl;
    cout << "* Original text: " << cx::trimBoth(text) << endl;
    cout << "* Transformed text: " << cx::toLowerCase(cx::trimBoth(text)) << endl;
    cout << endl;
    return 0;
}

int testCSVReader(const string& filename = "cx_example.csv")
{
    // Generate a CSV file
    ofstream file(filename);
    if (!file.is_open()) return -1;
    file << "Name, ID, Salary, Bonus" << endl;
    file << "SC, 2, 29.3, 2.8" << endl;
    file << "KL, 4, 18.10, 4.8" << endl;
    file << "NJ, 14, 27.10, 4.1" << endl;
    file << "WY, 16, 12.5, 6.1" << endl;
    file.close();

    // Read the CSV file
    cx::CSVReader reader;
    if (!reader.open(filename)) return -1;
    if (reader.size() != 5) return -1;
    if (reader.front().size() != 4) return -1;

    // Extract the data (with default arguments)
    cx::CSVReader::String2D all_with_header = reader.extString2D();
    if (all_with_header.size() != 5) return -1;
    if (all_with_header.front().size() != 4) return -1;

    cout << "### Test cx::CSVReader" << endl;
    for (size_t i = 0; i < all_with_header.size(); i++)
    {
        cout << "| ";
        for (size_t j = 0; j < all_with_header[i].size(); j++)
            cout << all_with_header[i][j] << " | ";
        cout << endl;
    }
    cout << endl;

    // Extract the data as specific types
    cx::CSVReader::String2D name = reader.extString2D(1, { 0 });
    cx::CSVReader::Int2D ids = reader.extInt2D(1, { 1 });
    cx::CSVReader::Double2D data = reader.extDouble2D(1, { 2, 3 });
    if (name.size() != 4 || ids.size() != 4 || data.size() != 4) return -1;
    if (name.front().size() != 1 || ids.front().size() != 1 || data.front().size() != 2) return -1;

    cout << "### Test cx::CSVReader" << endl;
    for (size_t i = 0; i < data.size(); i++)
        cout << "A person (name: " << name[i][0] << ", ID: " << ids[i][0] << ") will receive USD " << data[i][0] + data[i][1] << "." << endl;
    cout << endl;
    return 0;
}

int main()
{
    if (testStringOperations() < 0) return -1;
    if (testCSVReader() < 0) return -1;
    return 0;
}
