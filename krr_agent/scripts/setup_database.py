""" #!/usr/bin/env python3
Setup script to initialize TypeDB database with schema and data.
Run this script ONCE before running mission_planner.py

Prerequisites:
- TypeDB 2.28.3 server running on localhost:1729
- Run inside the course container (typedb-driver is already installed)
"""

import os
import re
import tempfile
from pathlib import Path

from typedb.driver import TypeDB, SessionType, TransactionType

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# True  -> Multiple PDDL -> keep predefined drop locations from data.tql + Task 3
# False -> Single   PDDL -> remove the marked drop-location block from data.tql before loading
MULTI_PDDL_FILE = os.getenv("USE_MULTI_PDDL", "true").lower() == "true"

DROP_START_MARKER = "# --------- DROP LOCATIONS START — OVERWRITE ONLY IF SINGLE DPPL ---------"
DROP_END_MARKER = "# --------- DROP LOCATIONS END — OVERWRITE ONLY IF SINGLE DPPL ---------"

def create_runtime_data_file(data_file_path: str) -> str:
    """
    Create a temporary copy of data.tql.
    If MULTI_PDDL_FILE is False, remove the marked drop-location block.
    """
    with open(data_file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    if not MULTI_PDDL_FILE:
        pattern = (
            re.escape(DROP_START_MARKER)
            + r".*?"
            + re.escape(DROP_END_MARKER)
        )
        content, substitutions = re.subn(pattern, "", content, flags=re.DOTALL)

        if substitutions == 0:
            print("[WARN] Drop-location marker block not found in data.tql. Loading original data unchanged.")
        else:
            print("[INFO] Removed predefined drop-location block from data.tql for single-PDDL mode.")
    else:
        print("[INFO] Keeping predefined drop locations from data.tql for multi-PDDL mode.")

    tmp_data_path = os.path.join(tempfile.gettempdir(), "data_runtime_generated.tql")
    with open(tmp_data_path, 'w', encoding='utf-8') as f:
        f.write(content)

    return tmp_data_path

def setup_database(address="localhost:1729", database_name="task_db"):
    """
    Create and populate the TypeDB database.
    
    Args:
        address: TypeDB server address
        database_name: Name for the database
    """
    print("="*60)
    print("TypeDB Database Setup")
    print("="*60)
    
    # Connect to TypeDB
    print(f"\nConnecting to TypeDB at {address}...")
    driver = TypeDB.core_driver(address)
    
    try:
        # Delete existing database if it exists
        if driver.databases.contains(database_name):
            print(f"Database '{database_name}' already exists. Deleting...")
            driver.databases.get(database_name).delete()
        
        # Create new database
        print(f"Creating database '{database_name}'...")
        driver.databases.create(database_name)
        
        # Load schema
        print("\nLoading schema from schema.tql...")
        with open(os.path.join(SCRIPT_DIR, "schema.tql"), 'r', encoding='utf-8') as f:
            schema_query = f.read()
        
        with driver.session(database_name, SessionType.SCHEMA) as session:
            with session.transaction(TransactionType.WRITE) as transaction:
                transaction.query.define(schema_query)
                transaction.commit()
        
        print("✓ Schema loaded successfully")
        
        # Load data
        print("\nLoading data from data.tql...")
        original_data_path = os.path.join(SCRIPT_DIR, "data.tql")
        runtime_data_path = create_runtime_data_file(original_data_path)

        with open(runtime_data_path, 'r', encoding='utf-8') as f:
            data_query = f.read()
        
        with driver.session(database_name, SessionType.DATA) as session:
            with session.transaction(TransactionType.WRITE) as transaction:
                transaction.query.insert(data_query)
                transaction.commit()
        
        print("✓ Data loaded successfully")
        
        # Verify setup by querying data
        print("\n" + "="*60)
        print("Verifying Database Setup")
        print("="*60)
        
        with driver.session(database_name, SessionType.DATA) as session:
            with session.transaction(TransactionType.READ) as transaction:
                # Count entities
                queries = [
                    ("Rooms", "match $x isa room; get $x; count;"),
                    ("Drop Locations", "match $x isa drop-location; get $x; count;"),
                    ("Scan Locations", "match $x isa scan-location; get $x; count;"),
                    ("Poses", "match $x isa pose; get $x; count;"),
                    ("Spatial Containments", "match $x isa spatial-containment; get $x; count;"),
                ]
                
                for name, query in queries:
                    result = transaction.query.get_aggregate(query)
                    # Handle Promise-based API in newer TypeDB drivers
                    if hasattr(result, "resolve"):
                        result = result.resolve()
                    if hasattr(result, "__iter__"):
                        result_list = list(result)
                    else:
                        result_list = [result]
                    if result_list:
                        value = result_list[0]
                        count = value.as_value().as_long() if hasattr(value, "as_value") else int(value)
                    else:
                        count = 0
                    print(f"{name}: {count}")
        
        print("\n" + "="*60)
        print("✓ Database setup complete!")
        print("="*60)
        
    except Exception as e:
        print(f"\n✗ Error during setup: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        driver.close()


if __name__ == "__main__":
    # Check if schema and data files exist
    if not os.path.exists(os.path.join(SCRIPT_DIR, "data.tql")):
        print("ERROR: data.tql not found!")
        print("Make sure you're running this script from the scripts directory.")
        exit(1)
    if not os.path.exists(os.path.join(SCRIPT_DIR, "schema.tql")):
        print("ERROR: schema.tql not found!")
        print("Make sure you're running this script from the scripts directory.")
        exit(1)

    setup_database()