#include <Arduino.h>
#include <SoftwareSerial.h>                                          // Подключаем библиотеку SoftwareSerial для общения с модулем по программной шине UART
SoftwareSerial softSerial(4,5);                                      // Создаём объект softSerial указывая выводы RX, TX (можно указывать любые выводы Arduino UNO)
                                                                     // В данном случае вывод TX модуля подключается к выводу 2 Arduino, а вывод RX модуля к выводу 3 Arduino.
//  Инициируем работу шин UART с указанием скоростей обеих шин:      //
void setup(){                                                        //
    softSerial.begin(115200);                                         // Инициируем передачу данных по программной шине UART на скорости 38400 (между модулем и Arduino)
        Serial.begin(115200);                                          // Инициируем передачу данных по аппаратной  шине UART на скорости  9600 (между Arduino и компьютером)
    Serial.println("start>");
}                                                                    //
                                                                     //
//  Выполняем ретрансляцию:                                          // Всё что пришло с модуля - отправляем компьютеру, а всё что пришло с компьютера - отправляем модулю
void loop(){                                                         //
    if(softSerial.available()){    Serial.write(softSerial.read());} // Передаём данные из программной шины UART в аппаратную  (от модуля     через Arduino к компьютеру)
    if(    Serial.available()){softSerial.write(    Serial.read());} // Передаём данные из аппаратной  шины UART в программную (от компьютера через Arduino к модулю    )

  /*softSerial.listen();
  Serial.println("Data from softSerial:");
  // while there is data coming in, read it
  // and send to the hardware serial port:
  while (softSerial.available() > 0) {
    char inByte = softSerial.read();
    Serial.write(inByte);
  }

  Serial.println();*/
}