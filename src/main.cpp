#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include "config.h"
#include "json.hpp"
#include "FusionEKF.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;
using std::vector;

// for convenience
using json = nlohmann::json;

/**
 * Checks if the SocketIO event has JSON data.
 * If there is data, the JSON object in string format will be returned,
 * else the empty string "" will be returned.
 * @param s string data
 * @return
 */
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    auto result = s.substr(b1, b2 - b1 + 1);
    if (DEVELOP_MODE) {
      std::cout << "\nreceived data: " << result <<"\n==========================================================================================" << std::endl;
    }
    return result;
  }
  return "";
}
/**
 * main function of EKF.
 * Read data from simulater, and call EKF, and return estimate x,y,rmse
 * @return
 */
int main() {
  // Create a uWebSocket instance
  uWS::Hub hub;

  // Create a Kalman Filter instance
  FusionEKF fusionEKF;
  // ekf calculate result
  vector<VectorXd> estimations;
  // real data
  vector<VectorXd> ground_truth;

  // An event listener to be called when a message is received from the server.
  hub.onMessage([&fusionEKF, &estimations, &ground_truth]
                    (uWS::WebSocket<uWS::SERVER> ws,
                     char *data,
                     size_t length,
                     uWS::OpCode opCode) {
    /**
     * "42" at the start of the message means there's a websocket message event.
     * The 4 signifies a websocket message
     * The 2 signifies a websocket event
     */
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto json_string = hasData(string(data));

      if (json_string != "") {
        auto json_object = json::parse(json_string);

        string event = json_object[0].get<string>();

        if (event == "telemetry") {
          // json[1] is the data JSON object
          string sensor_measurement = json_object[1]["sensor_measurement"];

          MeasurementPackage measurement_pack;
          std::istringstream iss(sensor_measurement);

          long long timestamp; // Integer number but at least 64 bit

          // reads first element from the current line
          string sensor_type;
          iss >> sensor_type;

          // parse json and build measurement package
          if (sensor_type.compare("L") == 0) {
            measurement_pack.sensor_type_ = MeasurementPackage::LASER;
            measurement_pack.raw_measurements_ = VectorXd(2);
            float px;
            float py;
            iss >> px;
            iss >> py;
            measurement_pack.raw_measurements_ << px, py;
            iss >> timestamp;
            measurement_pack.timestamp_ = timestamp;

          } else if (sensor_type.compare("R") == 0) {
            measurement_pack.sensor_type_ = MeasurementPackage::RADAR;
            measurement_pack.raw_measurements_ = VectorXd(3);
            float ro;
            float theta;
            float ro_dot;
            iss >> ro;
            iss >> theta;
            iss >> ro_dot;
            measurement_pack.raw_measurements_ << ro, theta, ro_dot;
            iss >> timestamp;
            measurement_pack.timestamp_ = timestamp;
          }

          float x_gt;
          float y_gt;
          float vx_gt;
          float vy_gt;
          iss >> x_gt;
          iss >> y_gt;
          iss >> vx_gt;
          iss >> vy_gt;

          VectorXd ground_truth_values(X_SIZE);
          ground_truth_values(0) = x_gt;
          ground_truth_values(1) = y_gt;
          ground_truth_values(2) = vx_gt;
          ground_truth_values(3) = vy_gt;
          ground_truth.push_back(ground_truth_values);

          // Call ProcessMeasurement(measurement_pack) for Kalman filter
          fusionEKF.ProcessMeasurement(measurement_pack);

          // Push the current estimated x,y position from the Kalman filter's to state vector
          VectorXd estimate(X_SIZE);

          double estimate_px = fusionEKF.ekf_.x_(0);
          double estimate_py = fusionEKF.ekf_.x_(1);
          double estimate_vx = fusionEKF.ekf_.x_(2);
          double estimate_vy = fusionEKF.ekf_.x_(3);

          estimate(0) = estimate_px;
          estimate(1) = estimate_py;
          estimate(2) = estimate_vx;
          estimate(3) = estimate_vy;

          estimations.push_back(estimate);

          // calculate RMSE
          VectorXd RMSE = Tools::CalculateRMSE(estimations, ground_truth);

          json msg_json;
          msg_json["estimate_x"] = estimate_px;
          msg_json["estimate_y"] = estimate_py;
          msg_json["rmse_x"] = RMSE(0);
          msg_json["rmse_y"] = RMSE(1);
          msg_json["rmse_vx"] = RMSE(2);
          msg_json["rmse_vy"] = RMSE(3);
          auto msg = "42[\"estimate_marker\"," + msg_json.dump() + "]";

          auto e_v = Tools::CalculateV(estimate);
          auto g_v = Tools::CalculateV(ground_truth_values);
          if (DEVELOP_MODE) {
            std::cout << "Estimate v is: " << e_v << ",\t real v: " << g_v << "\t diff: "
                      << (g_v - e_v) << std::endl;
          }

          // debug output
          if (DEVELOP_MODE) {
            std::cout << msg << std::endl;
          }
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }  // end "telemetry" if

      } else {
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if

  }); // end hub.onMessage

  hub.onConnection([&hub](uWS::WebSocket<uWS::SERVER> ws,
                          uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  hub.onDisconnection([&hub](uWS::WebSocket<uWS::SERVER> ws, int code,
                             char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected!!!" << std::endl;
  });

  int port = 4567;
  if (hub.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  hub.run();
}