package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

public class RingBuffer<T> {
    private List<T> store;
    private int nextIndex = 0;
    private int capacity;
    private boolean hasLooped = false;

    public RingBuffer(int size) {
        if (size < 1) throw new IllegalArgumentException();
        this.store = new ArrayList<T>(size);
        this.capacity = size;
    }

    public T add(T value) {
        T out = null;
        try {
            out = store.get(nextIndex);
            store.set(nextIndex, value);
            //System.out.println("Replacing index " + nextIndex + " value " + store.get(nextIndex) + " with " + value);
        } catch (Exception e) {
            store.add(value);
            //System.out.println("Adding index " + nextIndex + " value " + value);
        }
        nextIndex += 1;
        if (nextIndex >= capacity) {
            nextIndex = 0;
            hasLooped = true;
        }
        return out;
    }

    public boolean isFull() {
        return hasLooped;
    }

    public void rewind(int n) {
        if (n < 0) throw new IllegalArgumentException("Cannot rewind into the future, silly >:(");
        nextIndex -= n;
        nextIndex %= capacity;
        //System.out.println("Rewinding " + n + " positions: next is " + nextIndex);
    }
    public void rewind() {
        rewind(1);
    }

    public List<T> asList() { return store; }
    public Stream<T> asStream() { return store.stream(); }
    public T get(int i) { return store.get(i); }
    public int size() { return hasLooped ? capacity : nextIndex; }
    public int capacity() { return capacity; }

    public String toString() {
        return String.join(", ", asStream().<CharSequence>map(x -> "" + x).toList());
    }
}
