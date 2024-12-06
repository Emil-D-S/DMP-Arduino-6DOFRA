


/*
void readHandleCommands() {
  String serialData;
  String parsedData[32];
  
  if (Serial.available() > 0) {
    //readPos = Serial.parseInt();
    serialData = Serial.readString();
    parseString(serialData, parsedData, 32);


    if (parsedData[0] == "cp") {
      base.kp = parsedData[1].toFloat();
    }
    if (parsedData[0] == "cd") {
      base.kd = parsedData[1].toFloat();
    }
    if (parsedData[0] == "pos") {
      readPos = parsedData[1].toFloat();
    }
    Serial.read();
  }
}
*/

void separateTextAndNumber(String input, String& textPart, String& numberPart) {
  input.trim();  // Remove any leading/trailing whitespace or newline characters

  textPart = "";
  numberPart = "";

  bool foundNumber = false;
  bool foundDecimalPoint = false;

  for (unsigned int i = 0; i < input.length(); i++) {
    char ch = input[i];

    // Handle digits and the first decimal point found
    if (isDigit(ch) || (ch == '.' && !foundDecimalPoint && foundNumber)) {
      numberPart += ch;
      foundNumber = true;  // Mark that we've found the start of the number
      if (ch == '.') {
        foundDecimalPoint = true;  // Mark that we've found a decimal point
      }
    }
    // Handle minus sign only if it's the first character in the number part
    else if (ch == '-' && !foundNumber) {
      numberPart += ch;
    }
    // All other characters are considered part of the text
    else {
      textPart += ch;
    }
  }
}




// Function to parse the special string format into an array of substrings
int parseSpecialString(String input, String parts[], int maxParts) {
  int partIndex = 0;
  int startIndex = 0;

  // Loop through each character in the input string
  for (unsigned int i = 0; i < input.length(); i++) {
    char c = input.charAt(i);

    // Check for the end of a letter block or a number block
    if ((i > 0 && isDigit(c) && isAlpha(input.charAt(i - 1))) || (i > 0 && isAlpha(c) && isDigit(input.charAt(i - 1)))) {
      if (partIndex < maxParts) {
        parts[partIndex] = input.substring(startIndex, i);
        partIndex++;
        startIndex = i;
      }
    }
  }

  // Add the last part
  if (partIndex < maxParts) {
    parts[partIndex] = input.substring(startIndex);
    partIndex++;
  }

  return partIndex;  // Return the number of parts parsed
}

int parseString(String input, String parts[], int maxParts) {
  int partIndex = 0;

  for (unsigned int i = 0; i < input.length(); i++) {
    char c = input.charAt(i);

    // Check if the character is a letter or symbol
    if (isAlpha(c) || c == '$') {
      if (partIndex < maxParts) {
        parts[partIndex] = String(c);
        partIndex++;
      }
    }
    // If the character is a digit, accumulate all digits into a single part
    else if (isDigit(c)) {
      String number = "";
      while (isDigit(input.charAt(i)) && i < input.length()) {
        number += input.charAt(i);
        i++;
      }
      if (partIndex < maxParts) {
        parts[partIndex] = number;
        partIndex++;
      }
      i--;  // Step back one index because the for loop will increment it
    }
  }

  return partIndex;  // Return the number of parts parsed
}
