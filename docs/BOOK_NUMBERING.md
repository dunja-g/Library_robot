# Fixed-Grid Book Numbering

The default demonstration does not require a marker on a box or book. Each
book is assigned a database location in this format:

```text
<BOX>-L<LAYER>-P<POSITION>
```

For example, `1A-L3-P21` means:

- `1`: first row measured from Dock;
- `A`: left column when the robot faces forward from Dock;
- `L3`: layer 3 inside that box or shelf;
- `P21`: book position 21 on that layer.

The photographed scene is numbered:

```text
             far end

       3A              3B
       2A              2B
       1A              1B

              Dock
```

`Deep Learning` is currently configured as:

```python
"Deep Learning": {
    "book_id": "BK001",
    "box_id": "1A",
    "layer": 3,
    "position": 21,
    "location_code": "1A-L3-P21",
}
```

After a student QR check-in, the web UI asks the user to select a book. The
server:

1. validates the student session and loan status;
2. looks up the book record;
3. validates grid and encoder calibration;
4. creates a pending mission without writing a loan;
5. drives to the box without marker scanning;
6. displays the exact layer and position at `ARRIVED`;
7. records the loan only after pickup confirmation;
8. reverses safely to Dock and clears the student session.

Layer and position identify the book for the user; the mobile base navigates
only to the corresponding box. A lifting or picking mechanism would be needed
for the robot itself to retrieve a specific layer or position.

Add real placements in `pi/book_db.py`. Grid mode only lists books that contain
a `box_id`, so unassigned books cannot accidentally send the robot elsewhere.
