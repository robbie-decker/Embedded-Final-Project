#include<iostream>
#include<vector>

using namespace std;

int main()
{

    vector<int> v;
    cout << "Enter a list of postive numbers" << endl;
    cout << "Enter a negative number to end" << endl;

    int next;
    cin >> next;
    while(next > 0)
    {
        v.push_back(next);
        cout << next << "added";
        cout << "v.size() = " << v.size() << endl;
        cin >> next;

    }

    cout << "You've entered a: " << endl;
    for(unsigned int i = 0; i <v.size(); i++)
        cout << v[1] << " ";
    cout << endl;

    return 0;
}