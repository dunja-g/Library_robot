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

       4A              4B
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

The web UI still asks the user to select a book title. In grid mode the server:

1. looks up the book record;
2. reads `box_id`, `layer`, and `position`;
3. generates the encoder/IMU route to that box;
4. displays the full location code;
5. drives directly to the box without marker scanning;
6. announces arrival and returns to Dock.

Layer and position identify the book for the user; the mobile base navigates
only to the corresponding box. A lifting or picking mechanism would be needed
for the robot itself to retrieve a specific layer or position.

Add real placements in `pi/book_db.py`. Grid mode only lists books that contain
a `box_id`, so unassigned books cannot accidentally send the robot elsewhere.
