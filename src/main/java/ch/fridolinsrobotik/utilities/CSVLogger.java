package ch.fridolinsrobotik.utilities;

import java.io.FileWriter;
import java.io.IOException;
import java.util.*;

import edu.wpi.first.wpilibj.DriverStation;

public class CSVLogger implements AutoCloseable {
    private HashMap<String, List<String>> data;
    private HashMap<String, Integer> headerIndices = new HashMap<>();
    private FileWriter csvFile;
    private String filePath;
    private int indexOfNewHeader = 0;

    public CSVLogger(String filePath, String... headers) {
        this.filePath = filePath;
        String[] headersCopy = headers.clone();
        data = new HashMap<>();
        for (var header : headersCopy)
            data.put(header, new ArrayList<String>());
        open();
    }

    public CSVLogger(String filePath) {
        this.filePath = filePath;
        data = new HashMap<>();
        open();
    }

    public void put(String header, String... data) {
        if (!this.data.containsKey(header)) {
            headerIndices.put(header, indexOfNewHeader);
            this.data.put(header, new ArrayList<>());
            indexOfNewHeader++;
        }
        for (var d : data)
            this.data.get(header).add(d);
    }

    public void put(String header, double... data) {
        if (!this.data.containsKey(header)) {
            headerIndices.put(header, indexOfNewHeader);
            this.data.put(header, new ArrayList<>());
            indexOfNewHeader++;
        }
        for (var d : data)
            this.data.get(header).add(Double.toString(d));
    }

    public void put(String header, float... data) {
        if (!this.data.containsKey(header)) {
            headerIndices.put(header, indexOfNewHeader);
            this.data.put(header, new ArrayList<>());
            indexOfNewHeader++;
        }
        for (var d : data)
            this.data.get(header).add(Float.toString(d));
    }

    public void put(String header, int... data) {
        if (!this.data.containsKey(header)) {
            headerIndices.put(header, indexOfNewHeader);
            this.data.put(header, new ArrayList<>());
            indexOfNewHeader++;
        }
        for (var d : data)
            this.data.get(header).add(Integer.toString(d));
    }

    public void put(String header, short... data) {
        if (!this.data.containsKey(header)) {
            headerIndices.put(header, indexOfNewHeader);
            this.data.put(header, new ArrayList<>());
            indexOfNewHeader++;
        }
        for (var d : data)
            this.data.get(header).add(Short.toString(d));
    }

    public void put(String header, byte... data) {
        if (!this.data.containsKey(header)) {
            headerIndices.put(header, indexOfNewHeader);
            this.data.put(header, new ArrayList<>());
            indexOfNewHeader++;
        }
        for (var d : data)
            this.data.get(header).add(String.format("%02x", d));
    }

    public void put(String header, boolean... data) {
        if (!this.data.containsKey(header)) {
            headerIndices.put(header, indexOfNewHeader);
            this.data.put(header, new ArrayList<>());
            indexOfNewHeader++;
        }
        for (var d : data)
            this.data.get(header).add(Boolean.toString(d));
    }

    private static int max(Integer[] numbers) {
        int max = numbers[0];
        for (int i = 1; i < numbers.length; i++)
            if (numbers[i] > max)
                max = numbers[i];
        return max;
    }

    public void writeToFile() {
        List<List<String>> table = new ArrayList<>(data.keySet().size()); // list of rows
        for (int i = 0; i < data.keySet().size(); i++)
            table.add(new ArrayList<>());

        for (var header : data.keySet()) {
            table.get(headerIndices.get(header)).add(header);
            table.get(headerIndices.get(header)).addAll(data.get(header));
        }
        int maxColumnSize = 0;
        if (table.size() != 0)
            maxColumnSize = max(table.stream().map(List::size).toArray(Integer[]::new));
        StringJoiner[] rows = new StringJoiner[maxColumnSize];
        for (int i = 0; i < rows.length; i++)
            rows[i] = new StringJoiner(", ");

        for (int i = 0; i < maxColumnSize; i++) {
            for (var column : table)
                if (i < column.size())
                    rows[i].add(column.get(i));
        }

        StringJoiner stringTable = new StringJoiner("\n");
        for (var row : rows)
            stringTable.add(row.toString());
        try {
            csvFile.write(stringTable.toString());
        } catch (IOException e) {
            DriverStation.getInstance().reportError(
                    "An IOException occurred while writing to the csv file, with error message: " + e.getMessage(),
                    true);
            System.exit(-1);
        }
    }

    @Override
    public void close() {
        try {
            csvFile.close();
        } catch (IOException e) {
            DriverStation.getInstance().reportError(
                    "An IOException occurred while closing the csv file, with error message: " + e.getMessage(), true);
            System.exit(-1);
        }
    }

    public void open() {
        try {
            csvFile = new FileWriter(filePath);
        } catch (IOException e) {
            DriverStation.getInstance().reportError(
                    "An IOException occurred while opening the csv file, with error message: " + e.getMessage(), true);
            System.exit(-1);
        }
    }
}
