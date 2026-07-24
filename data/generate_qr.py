"""
Generate QR codes for all students in students.json.
Saves PNG files in the data/ directory.
"""
import os
import json

try:
    import qrcode
except ImportError:
    print("Error: The 'qrcode' library is not installed.")
    print("Please run: pip install qrcode[pil]")
    exit(1)

def main():
    data_dir = os.path.dirname(os.path.abspath(__file__))
    students_file = os.path.join(data_dir, 'students.json')
    
    if not os.path.exists(students_file):
        print(f"Error: Could not find {students_file}")
        return

    with open(students_file, 'r') as f:
        students = json.load(f)

    for student in students:
        student_id = student.get('id')
        qr_code_text = student.get('qr_code')
        
        if not student_id or not qr_code_text:
            continue
            
        img = qrcode.make(qr_code_text)
        filename = f"qr_{student_id}.png"
        filepath = os.path.join(data_dir, filename)
        
        img.save(filepath)
        print(f"Generated {filename} for student {student.get('name')}")

if __name__ == "__main__":
    main()
