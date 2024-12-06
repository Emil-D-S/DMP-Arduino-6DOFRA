void printRepeated(const char* str, int count) {
  String strToPrint = "";
  for (int i = 0; i < count; i++) {
    strToPrint += str; // Use '+=' to append the string
  }
  Serial.print(strToPrint); // Print the accumulated string
}