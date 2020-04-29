#ifndef LOAD_FLOW_GAUSS_SEIDEL
#define LOAD_FLOW_GAUSS_SEIDEL

#include <vector>
#include <complex>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#define width 10
#define YbusWidth 22 // Ybus values have kind of larger width
#define PI 3.14159265358979323846

using namespace std;

class load_flow
{
private:
    int iterations, N;
    string header, busInputData, lineInputData; //size of busData
    vector<vector<double>> busData, lineData;   // 0 base indexing
    vector<complex<double>> S;                  // complex power , 1 base indexing
    vector<vector<complex<double>>> V;          // voltages , 1 base indexing
    vector<vector<complex<double>>> Y;

    void read();
    void init_S(); //initialising S // busdata, S, N
    void init_V(); // initialising V // busData, V, N
    auto guass_seidel(int, int, bool);
    void update();
    void output();
    auto reactive_power(int, int);
    auto real_power(int, int);

public:
    load_flow(string, string, int);
    void calculateYBus();
    void printBusData();
    void printLineData();
    void printYbus();
    void solveLoadFlow();
};
/* 
bus input data format:
    type of bus, V, δ, Pg, Qg, Pl ,Ql ,Qg_max, Qg_min
    type of bus is an integer 1-3
    1: slack (there can be only one slack bus)
    2: PV / Gen bus
    3: PQ / Load bus
line input data format:
    from bus, to bus, R, X,G,B, Max MVA
Assumptions:
    * Bus1 is slack bus
    * G=0;
*/
load_flow::load_flow(string busInputData, string lineInputData, int iter)
{
    this->iterations = iter;
    this->busInputData = busInputData;
    this->lineInputData = lineInputData;
    this->read();
    this->N = this->busData.size();
    this->init_S();
    this->init_V();
}

void load_flow::read()
{
    string line, word;
    vector<double> row;

    fstream fin;
    fin.open(this->busInputData, ios::in);
    if (!fin.is_open())
    {
        cout << "Bus input data file does not exist. Remember to add file name with extension\n";
        exit(0);
    }
    else
    {
        getline(fin, line, '\n');
        this->header = line;
        int j = 0;
        while (getline(fin, line, '\n'))
        {
            stringstream s(line);
            while (getline(s, word, ','))
            {
                if (word == "")
                    row.push_back(0);
                else
                    row.push_back(stod(word));
            }
            this->busData.push_back(row);
            j++;
            row.clear();
        }
        fin.close();
    }
    fin.open(this->lineInputData, ios::in);
    if (!fin.is_open())
    {
        cout << "line input data file does not exist. Remember to add file name with extension\n";
        exit(0);
    }
    else
    {
        getline(fin, line, '\n');
        int i = 0;
        while (getline(fin, line, '\n'))
        {
            stringstream s(line);
            while (getline(s, word, ','))
            {
                if (i == 4) // assuming G= 0 hence not pushing it into LineData
                {
                    i++;
                    continue;
                }
                row.push_back(stod(word));
                i++;
            }
            i = 0;
            this->lineData.push_back(row);
            row.clear();
        }
        fin.close();
    }
}
void load_flow::init_S()
{
    S = vector<complex<double>>(N + 1, 0);
    for (int i = 0; i < this->N; i++)
    {
        switch ((int)busData[i][0])
        {
        case 2: // PV Bus
            this->S[i + 1].real(this->busData[i][3] - this->busData[i][5]);
            break;
        case 3: // PQ Bus
            this->S[i + 1].real(this->busData[i][3] - this->busData[i][5]);
            this->S[i + 1].imag(this->busData[i][4] - this->busData[i][6]);
            break;
        default:
            break;
        }
    }
}
/* prints line data */
void load_flow::printLineData()
{
    for (int i = 0; i < this->lineData.size(); i++)
    {
        for (int j = 0; j < this->lineData[i].size(); j++)
            cout << left << setw(width) << setfill(' ') << this->lineData[i][j];
        cout << "\n";
    }
}

void load_flow::init_V()
{
    V = vector<vector<complex<double>>>(N + 1);
    this->V[1].push_back(complex<double>(1, 0));
    for (int i = 1; i < this->N; i++)
    {
        switch ((int)this->busData[i][0])
        {
        case 2: // PV BUS
            this->V[i + 1].push_back(complex<double>(this->busData[i][1], 0));
            break;

        case 3: // PQ BUS
            this->V[i + 1].push_back(complex<double>(1, 0));
            break;
        default:
            break;
        }
    }
}
/* prints bus data. data is updated after calling solveLoadFlow() */
void load_flow::printBusData()
{
    for (int i = 0; i < this->N; i++)
    {
        cout << "bus " << i + 1 << "   ";
        for (int j = 1; j < this->busData[i].size(); j++)
            cout << left << setw(width) << setfill(' ') << this->busData[i][j];
        cout << "\n";
    }
}
/*calculates the Y Bus matrix */
void load_flow::calculateYBus()
{
    Y = vector<vector<complex<double>>>(N + 1, vector<complex<double>>(N + 1, 0));
    complex<double> y1, y2, one(1, 0);
    y2.real(0);
    for (int i = 0; i < this->lineData.size(); i++)
    {
        y1.real(this->lineData[i][2]);
        y1.imag(this->lineData[i][3]);
        y1 = one / y1;
        y2.imag(this->lineData[i][4] / 2);

        this->Y[this->lineData[i][0]][this->lineData[i][1]] -= y1;
        this->Y[this->lineData[i][1]][this->lineData[i][0]] -= y1;
        this->Y[this->lineData[i][0]][this->lineData[i][0]] += (y1 + y2);
        this->Y[this->lineData[i][1]][this->lineData[i][1]] += (y1 + y2);
    }
}
/* Print the Y Bus matrix */
void load_flow::printYbus()
{
    for (int i = 1; i <= this->N; i++)
    {
        for (int j = 1; j <= this->N; j++)
            cout << left << setw(YbusWidth) << setfill(' ') << this->Y[i][j];
        cout << "\n";
    }
}
auto load_flow::guass_seidel(int itr, int k, bool flag)
{
    complex<double> sum(0, 0);
    for (int n = 1; n <= this->N; n++)
    {
        if (n < k)
            sum += this->Y[k][n] * this->V[n][itr];
        else if (n > k)
            sum += this->Y[k][n] * this->V[n][itr - 1];
    }
    if (!flag) // the above equation can be applied two times in a single iteration (using V[k][itr-1] & then using V[k][itr])
        return (conj(this->S[k] / this->V[k][itr - 1]) - sum) / (this->Y[k][k]);
    return (conj(this->S[k] / this->V[k][itr]) - sum) / (this->Y[k][k]);
}
auto load_flow::reactive_power(int itr, int k)
{
    double mag{abs(this->V[k][itr])};
    double angle{arg(this->V[k][itr])};
    double sum{0};
    for (int n = 1; n <= this->N; n++)
        sum += abs(this->V[n][itr]) * (this->Y[k][n].real() * sin(angle - arg(this->V[n][itr])) - this->Y[k][n].imag() * cos(angle - arg(this->V[n][itr])));

    sum *= mag;
    return sum;
}
auto load_flow::real_power(int itr, int k)
{
    double mag{abs(this->V[k][itr])};
    double angle{arg(this->V[k][itr])};
    double sum{0};
    for (int n = 1; n <= this->N; n++)
        sum += abs(this->V[n][itr]) * (this->Y[k][n].real() * cos(angle - arg(this->V[n][itr])) + this->Y[k][n].imag() * sin(angle - arg(this->V[n][itr])));
    sum *= mag;
    return sum;
}
void load_flow::update()
{
    for (int i = 1; i <= this->N; i++)
    {
        this->busData[i - 1][1] = abs(this->V[i][this->iterations]);            // V
        this->busData[i - 1][2] = arg(this->V[i][this->iterations]) * 180 / PI; // delta in degrees
        this->busData[i - 1][3] = this->S[i].real() + this->busData[i - 1][5];  // Pg
        this->busData[i - 1][4] = this->S[i].imag() + this->busData[i - 1][6];  // Qg
    }
}
void load_flow::output()
{
    fstream fout;
    fout.open("output.csv", ios::out);
    fout << this->header << "\n";
    for (int i = 0; i < this->N; i++)
    {
        for (int j = 0; j < this->busData[i].size(); j++)
            fout << this->busData[i][j] << ",";
        fout << "\n";
    }
}
/* solves the load flow problem using gauss-seidel iterative method.
    updates the bus data.
    final result is stored in output.csv
 */
void load_flow::solveLoadFlow()
{
    bool type_change;
    double Q, Qg;
    // logic
    for (int i = 1; i <= this->iterations; i++)
    {
        this->V[1].push_back(complex<double>(1, 0));
        for (int j = 2; j <= this->N; j++)
        {
            switch ((int)this->busData[j - 1][0])
            {
            case 2: // pv // Load
                type_change = true;
                Q = this->reactive_power(i - 1, j);
                Qg = Q + this->busData[j - 1][6]; //  Qg = Q + Ql

                if (Qg > this->busData[j - 1][7]) // Qg > Qgmax
                    Q = this->busData[j - 1][7];
                else if (Qg < this->busData[j - 1][8]) // Qg < Qgmin
                    Q = this->busData[j - 1][8];
                else
                    type_change = false;
                this->S[j].imag(Q);
                this->V[j].push_back(this->guass_seidel(i, j, 0));
                if (!type_change) // Qg does not exceed its limits
                {
                    this->V[j][i] = polar(abs(this->V[j][0]), arg(this->V[j][i]));
                    this->V[j][i] = this->guass_seidel(i, j, 1);
                    this->V[j][i] = polar(abs(this->V[j][0]), arg(this->V[j][i]));
                }
                else // Qg exceeded its limits
                    this->V[j][i] = this->guass_seidel(i, j, 1);
                break;

            case 3: // pq // Gen
                this->V[j].push_back(this->guass_seidel(i, j, 0));
                this->V[j][i] = this->guass_seidel(i, j, 1);
                break;
            default:
                break;
            }
        }
    }

    // P and Q for slack bus
    this->S[1].real(this->real_power(this->iterations, 1));
    this->S[1].imag(this->reactive_power(this->iterations, 1));
    // end of logic

    this->update(); // updating busData
    this->output();
}
#endif