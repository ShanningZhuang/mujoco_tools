## Data Input Rule

The standard data input should adhere to the following specifications:

*   **File Format:** `.npy` (NumPy array file)
*   **Data Structure:**
    *   The **1st dimension** (rows) of the array must represent the **time stamp**.
    *   The **2nd dimension** (columns) of the array must represent the **data** itself.

### Example:

If you have `T` time steps and `D` data features, the NumPy array loaded from the `.npy` file should have a shape of `(T, D)`.
