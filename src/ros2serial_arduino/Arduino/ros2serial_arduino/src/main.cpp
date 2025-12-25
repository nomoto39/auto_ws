#include <Arduino.h>


void send_serial(int left, int right){
  Serial.print(left);
  Serial.print(",");
  Serial.print(right);
  Serial.println();
  delay(200);
}


void receive_serial() {
    if (Serial.available() > 0) 
    {
        String linear_x_str;
        String angular_z_str;
        String cylinder_1_str;
        String cylinder_2_str;

        float linear_x;
        float angular_z;
        float cylinder_1;
        float cylinder_2;

        String received_data = Serial.readStringUntil('\n');    // 改行までのデータを読み込む
        Serial.println(received_data);                          // データをシリアルモニターに表示

        int comma_index1 = received_data.indexOf(',');                                        // 1番目のカンマの位置を検索
        if (comma_index1 != -1)
        {
            linear_x_str = received_data.substring(0, comma_index1);                            // 1番目の部分文字列を取得
            int comma_index2 = received_data.indexOf(',', comma_index1 + 1);                  // 2番目のカンマの位置を検索
            if (comma_index2 != -1)
            {
                angular_z_str = received_data.substring(comma_index1 + 1);                      // 2番目の部分文字列を取得
                int comma_index3 = received_data.indexOf(',', comma_index2 + 1);              // 3番目のカンマの位置を検索
                if (comma_index3 != -1)
                {
                    cylinder_1_str = received_data.substring(comma_index2 + 1, comma_index3);   // 3番目の部分文字列を取得
                    cylinder_2_str = received_data.substring(comma_index3 + 1);                 // 4番目の部分文字列を取得
                }
                else
                {
                    // カンマが3つ未満の場合のエラーハンドリング
                    Serial.println("Error: Less than 2 commas in the inputString.");
                }
            }
            else
            {
                // カンマが2つ未満の場合のエラーハンドリング
                Serial.println("Error: Less than 2 commas in the inputString.");
            }
        }
        else
        {
            // カンマが見つからない場合のエラーハンドリング
            Serial.println("Error: No comma found in the inputString.");
        }

        linear_x = linear_x_str.toFloat();
        angular_z = angular_z_str.toFloat();
        cylinder_1 = cylinder_1_str.toFloat();
        cylinder_2 = cylinder_2_str.toFloat();

        // データをシリアルモニターに表示
        Serial.print("linear.x: ");
        Serial.print(linear_x);
        Serial.print(" angular.z: ");
        Serial.print(angular_z);
        Serial.print(" cylinder_1: ");
        Serial.print(cylinder_1);
        Serial.print(" cylinder_2: ");
        Serial.println(cylinder_2);
    }
}


void setup() {
    Serial.begin(115200);
}


void loop() {
    receive_serial(); // uncomment out to recieve serial data
    // send_serial(); // uncomment out to send serial data
    
}