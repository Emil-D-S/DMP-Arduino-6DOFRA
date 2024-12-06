//const float PI = 3.14159265358979323;

float calculateMean(float data[], int size) {
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += data[i];
    }
    return sum / size;
}

// Function to calculate standard deviation of an array
float calculateStdDev(float data[], int size, float mean) {
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += pow(data[i] - mean, 2);
    }
    return sqrt(sum / size);
}

// Function to calculate the average ignoring outliers
float calculateAvgWithoutOutliers(float data[], int size, float kthr) {
    float mean = calculateMean(data, size);
    float stdDev = calculateStdDev(data, size, mean);

    // Threshold to ignore outliers (e.g., 2 standard deviations)
    float threshold = kthr * stdDev;

    float sum = 0;
    int count = 0;

    // Calculate sum and count of values within the threshold
    for (int i = 0; i < size; i++) {
        if (fabs(data[i] - mean) <= threshold) {
            sum += data[i];
            count++;
        }
    }

    // Return the average without outliers
    return count > 0 ? sum / count : 0;
}