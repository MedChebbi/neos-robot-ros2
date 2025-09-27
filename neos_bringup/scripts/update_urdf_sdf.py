#!/usr/bin/env python3
"""
Script to update SDF from XACRO while preserving existing plugins.

This script:
1. Converts XACRO to URDF
2. Converts URDF to SDF
3. Extracts plugins from existing SDF
4. Merges new SDF content with preserved plugins

Usage:
    python3 update_urdf_sdf.py [--backup]
    
Default paths (relative to workspace root):
    XACRO: neos_description/urdf/neos.xacro
    URDF:  neos_description/urdf/neos.urdf  
    SDF:   neos_gazebo/models/neos.sdf
"""

import argparse
import subprocess
import sys
from pathlib import Path
from typing import List, Optional
import xml.etree.ElementTree as ET
from xml.dom import minidom


def run_command(cmd: List[str], cwd: Optional[Path] = None) -> str:
    """Run a command and return its output."""
    try:
        result = subprocess.run(
            cmd, 
            capture_output=True, 
            text=True, 
            check=True, 
            cwd=cwd
        )
        return result.stdout
    except subprocess.CalledProcessError as e:
        print(f"Error running command {' '.join(cmd)}: {e}")
        print(f"stderr: {e.stderr}")
        sys.exit(1)


def convert_xacro_to_urdf(xacro_path: Path, urdf_path: Path) -> None:
    """Convert XACRO file to URDF."""
    print(f"Converting {xacro_path} to {urdf_path}")
    cmd = ["xacro", str(xacro_path)]
    urdf_content = run_command(cmd)
    
    with open(urdf_path, 'w') as f:
        f.write(urdf_content)
    print(f"✓ URDF generated: {urdf_path}")


def convert_urdf_to_sdf(urdf_path: Path, sdf_path: Path) -> None:
    """Convert URDF file to SDF."""
    print(f"Converting {urdf_path} to {sdf_path}")
    cmd = ["gz", "sdf", "-p", str(urdf_path)]
    sdf_content = run_command(cmd)
    
    with open(sdf_path, 'w') as f:
        f.write(sdf_content)
    print(f"✓ SDF generated: {sdf_path}")


def extract_plugins(sdf_path: Path) -> List[ET.Element]:
    """Extract plugin elements from existing SDF file."""
    if not sdf_path.exists():
        print(f"No existing SDF file at {sdf_path}, no plugins to preserve")
        return []
    
    try:
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        
        # Find all plugin elements
        plugins = []
        for plugin in root.findall('.//plugin'):
            plugins.append(plugin)
        
        print(f"✓ Found {len(plugins)} plugin(s) to preserve")
        return plugins
    except ET.ParseError as e:
        print(f"Warning: Could not parse existing SDF file {sdf_path}: {e}")
        return []


def merge_plugins_into_sdf(sdf_path: Path, plugins: List[ET.Element]) -> None:
    """Merge preserved plugins into the new SDF file."""
    if not plugins:
        print("No plugins to merge")
        return
    
    try:
        tree = ET.parse(sdf_path)
        root = tree.getroot()
        
        # Find the model element
        model = root.find('.//model')
        if model is None:
            print("Warning: No model element found in SDF, cannot add plugins")
            return
        
        # Add plugins to the model
        for plugin in plugins:
            model.append(plugin)
        
        # Write back to file with proper formatting
        rough_string = ET.tostring(root, encoding='unicode')
        reparsed = minidom.parseString(rough_string)
        pretty_xml = reparsed.toprettyxml(indent="  ")
        
        # Remove empty lines and fix formatting
        lines = [line for line in pretty_xml.split('\n') if line.strip()]
        formatted_xml = '\n'.join(lines)
        
        with open(sdf_path, 'w') as f:
            f.write(formatted_xml)
        
        print(f"✓ Merged {len(plugins)} plugin(s) into {sdf_path}")
        
    except ET.ParseError as e:
        print(f"Error parsing SDF file {sdf_path}: {e}")
        sys.exit(1)


def get_default_paths() -> tuple[Path, Path, Path]:
    """Get default file paths relative to workspace root."""
    # Find workspace root by looking for src directory
    current_dir = Path(__file__).parent
    workspace_root = None
    
    # Walk up the directory tree to find workspace root
    for parent in [current_dir] + list(current_dir.parents):
        if (parent / "neos_description" / "urdf").exists():
            workspace_root = parent
            break
    
    if not workspace_root:
        print("Error: Could not find workspace root (neos_description directory)")
        sys.exit(1)
    
    xacro_path = workspace_root / "neos_description" / "urdf" / "neos.xacro"
    urdf_path = workspace_root / "neos_description" / "urdf" / "neos.urdf"
    sdf_path = workspace_root / "neos_gazebo" / "models" / "neos.sdf"
    
    return xacro_path, urdf_path, sdf_path


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description="Update SDF from XACRO while preserving plugins",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 update_urdf_sdf.py                    # Use default paths
  python3 update_urdf_sdf.py --backup          # Create backup of existing SDF
  python3 update_urdf_sdf.py --xacro custom.xacro --sdf custom.sdf  # Custom paths
        """
    )
    
    # Optional arguments for custom paths
    parser.add_argument(
        "--xacro", 
        help="Path to XACRO file (default: neos_description/urdf/neos.xacro)"
    )
    parser.add_argument(
        "--urdf", 
        help="Path to output URDF file (default: neos_description/urdf/neos.urdf)"
    )
    parser.add_argument(
        "--sdf", 
        help="Path to output SDF file (default: neos_gazebo/models/neos.sdf)"
    )
    parser.add_argument(
        "--backup", 
        action="store_true",
        help="Create backup of existing SDF file"
    )
    
    args = parser.parse_args()
    
    # Get file paths (use defaults if not specified)
    if args.xacro or args.urdf or args.sdf:
        # Custom paths specified
        if not all([args.xacro, args.urdf, args.sdf]):
            print("Error: If specifying custom paths, you must provide --xacro, --urdf, and --sdf")
            sys.exit(1)
        xacro_path = Path(args.xacro)
        urdf_path = Path(args.urdf)
        sdf_path = Path(args.sdf)
    else:
        # Use default paths
        xacro_path, urdf_path, sdf_path = get_default_paths()
    
    # Validate input file
    if not xacro_path.exists():
        print(f"Error: XACRO file {xacro_path} does not exist")
        sys.exit(1)
    
    print("🤖 Updating NEOS robot SDF files...")
    print(f"XACRO: {xacro_path}")
    print(f"URDF:  {urdf_path}")
    print(f"SDF:   {sdf_path}")
    print()
    
    # Extract plugins from existing SDF BEFORE creating backup
    plugins = extract_plugins(sdf_path)
    
    # Create backup if requested
    if args.backup and sdf_path.exists():
        backup_path = sdf_path.with_suffix('.sdf.backup')
        sdf_path.rename(backup_path)
        print(f"✓ Created backup: {backup_path}")
    
    # Convert XACRO to URDF
    convert_xacro_to_urdf(xacro_path, urdf_path)
    
    # Convert URDF to SDF
    convert_urdf_to_sdf(urdf_path, sdf_path)
    
    # Merge preserved plugins into new SDF
    merge_plugins_into_sdf(sdf_path, plugins)
    
    print(f"\n🎉 Successfully updated {sdf_path} with preserved plugins!")


if __name__ == "__main__":
    main()
