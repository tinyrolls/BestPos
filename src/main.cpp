#include <iostream>
#include <fstream>
#include "PosComputator.h"

using namespace std;
int main() {
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", ",", "", "", "", "");
  Eigen::MatrixXd pos(4,3);
  pos <<  2.5552738, 11.279182, 2.382, //A20
          5.936005, 12.662466, 2.382,  //A16
          6.9418616, 9.349261, 2.385, //A18
          3.8766613, 7.131789, 2.383; //A17

  cout << "Test Sample: \n" << pos << endl;
  ifstream csv_data("../dataset/log_0224.txt", ios::in);
  ofstream filter_result("../dataset/result/filter0.csv", ios::out|ios::trunc);

  string filter_tag = "0";
  filter_result << "x_val,y_val,z_val,tag\n";
  PosComputator pc(1, false);

  
  if (!csv_data.is_open()) {
    cout << "Error file" << endl;
    exit(1);
  }

  string ling;
  while (std::getline(csv_data, ling)) {
    Eigen::VectorXd dis(4);
    stringstream ss(ling);
    string str;

    int tagId  = 0;
    int j = 0;
    while (getline(ss, str, ',') && j < 5) {
      if (j == 0) {
        tagId = stof(str);
      } else {
        dis(j-1) = stof(str);
      }
      j++;
    }


    Eigen::VectorXd result = pc.posNLLSwithAnchor(tagId, pos, dis);

    filter_result <<  result.format(CommaInitFmt) << "," << filter_tag << endl;

    cout << "Result " << result.format(CommaInitFmt) << endl;

  }
  
  return 0;

}