/* COPYRIGHt 2017 Andrew Prindle <prindle.andrew<_at_>gmail.com> */

/* To Reashearch
    Thin pointers vs Fat pointers
    Stopped reading the rustbyexample.com at attributes: rustbyexample.com/attribute/cfg.html
    
    rustbyexample.com/attributes/cfg.html
    https://getawesomeness.herokuapp.com/get/rust
*/



// Std stuff

use std::env;
use std::collections::HashMap;
use std::process::{Command, Output, ExitStatus};
use std::time::Instant;
use std::io;
use std::mem;



// File stuff
use std::io::{BufReader, BufRead, BufWriter, Write};
use std::io::{Write, Read};
use std::prelude::*;
use std::fs::File;
use std::path::{Path, PathBuf};
use std::ffi::OsStr;
use std::fs::{self, DirEntry};
use scan_dir::ScanDir;

// Useful and Frequently used crates
use ::serde::{Deserialize, Serialize};
use ::rmps::{Deserializer, Serializer};
use serde_json::Error;
use regex::Regex;
use chrono::NaiveDate;








// 
/*
    
    
    Log Output (trace/debug/info/warn/error levels)
    set RUST_LOG=my_crate=debug
    
    Testing Flags
    set RUST_TEST_THREADS=1
    set RUST_TEST_NOCAPTURE=1
    
    Supress Warning
    set RUSTFLAGS=-Awarnings
*/


// DOTENV - ENVIRONMENTAL VARIABLES FROM FILE
extern crate dotenv;
use dotenv::dotenv;
dotenv().ok();





// MODULES AND VISIBILITY - PUBLIC vs PRIVATE
/* Visibility Modifiers:
    pub
    pub (crate) - visible to current crate
    pub (in <ModulePath>) - makes visible to the parent module specified by path
    pub ([in] self) - visible within current module
    pub ([in] super) - visible to parent module

    private modules can only be accessed by current module or descendants
*/

// https://doc.rust-lang.org/reference/visibility-and-privacy.html
// Re-export a module as public so that other modules can view inside it
pub mod something;

pub mod outer_mod {
    pub mod inner_mod {
        // Visible within the outer_mod
        pub(in outer_mod) fn outer_mod_visible_fn() { /* ... */ }
        // Visible to the entire crate
        pub(crate) fn crate_visible_fn() { /* ... */ }
        // Visible within outer_mod
        pub(self) fn inner_mod_visible_fn() { /* ... */ }
        pub(super) fn super_mod_visible_fn() {
            // inner_mod_visible_fn() is visible since it is in the same module
            inner_mod_visible_fn();
        }
    }
    pub fn foo() {
        // These work:
        inner_mod::outer_mod_visible_fn();
        inner_mod::crate_visible_fn();
        inner_mod::super_mod_visible_fn();
        // This does not:
        //  inner_mod::inner_mod_visible_fn();
    }
}
fn bar() {
    // This works fine
    outer_mod::foo();
    // Still visible since it's in the same crate
    outer_mod::inner_mod::crate_visible_fn();
    // Do not work:
    //  outer_mod::inner_mod::super_mod_visible_fn();  fails because not in outer module
    //  outer_mod::inner_mod::outer_mod_visible_fn();  fails because outside of outer module
    //  outer_mod::inner_mod::inner_mod_visible_fn();  fails because not in the same module
    
}

// Re-exports
// reexport the api module to make it publicly accessable
pub use self::implementation::api;
mod implementation {
    pub mod api {
        pub fn f() { /* ... */ }
    }
}
// Valid
api::f() 
// Invalid
// implementation::api:f(); // Privacy violation - implementation is not public

#[path = "some_thing"]
mod thing;

use some as thing;
use a::b::{self as ab, c as abc};
use a::b::*;

// Use Examples:
// https://docs.rust-lang.org/reference/items.html#use-declarations
use foo::baz::foobaz;    // good: foo is at the root of the crate

mod foo {
    mod example {
        pub mod iter {}
    }
    use foo::example::iter; // good: foo is at crate root
//  use example::iter;      // bad:  example is not at the crate root
    use self::baz::foobaz;  // good: self refers to module 'foo'
    use foo::bar::foobar;   // good: foo is at crate root
    pub mod bar {
        pub fn foobar() { }
    }
    pub mod baz {
        use super::bar::foobar; // good: super refers to module 'foo'
        pub fn foobaz() { }
    }
}

//











// TESTING
// To show println output: cargo test -- --color always --nocapture
//    cargo test -- --nocapture
//   Or use: set RUST_TEST_NOCAPTURE=1
// the tests dir treats all rust files in it as a test, you must still use the #[test]
#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
    }
    
    #[test]
    #[should_panic]
    // or use:
    #[should_panic(expected = "Guess value must be less than or equal to 100")]
    fn it_works2() {
    }
    #[test]
    #[ignore]
    // run ignored tests with cargo test -- --ignored
    fn it_works2() {
    }
}

// test single item by specifing its name like:
//     cargo test test_a
// or specify part of the name and it will match part of the test name
//     cargo test test_


// Setting Testing Concurrency (defaults number of threads to number of coress)
// set RUST_TEST_THREADS=1
// cargo test -- --test-threads=1 





// BENCHMARKING
// http://siciarz.net/24-days-rust-static-initialization/

#[cfg(test)]
mod tests {
    use super::*;
    use test::Bencher;

    #[bench]
    fn bench_match_lookup(b: &mut Bencher) {
        b.iter(|| find_color("White"))
    }

    #[bench]
    fn bench_lazy_static_map(b: &mut Bencher) {
        b.iter(|| find_color_lazy_static("White"))
    }

    #[bench]
    fn bench_phf_map(b: &mut Bencher) {
        b.iter(|| find_color_phf("White"))
    }
}





// DOCUMENTATION
// //! is module level documentation
// /// is function level documentatino
// /// This function adds two to its argument.
// ///
// /// # Examples
// ///
// /// ```
// /// use adder::add_two;
// ///
// /// assert_eq!(4, add_two(2));
// /// ```




// UNIMPLEMENTED AND DIVERGING FUNCTIONS
// Unimplemented
unimplemented!() // specifies an unimplemented item

// Diverging Functions
fn my_error(some: &str) -> ! {
    // this is called a diverging function because it never returns a value to the caller
    // Every control path in a diverging function must end in:
    //    panic!()
    //    a loop with no break
    //    a call to another diverging function
    // ! does not denote a type
}
fn maybe_error(i: u32) -> u32 {
    if i == 666 {
        return 666;
    } else {
        my_error(); // If my_error() did not have a ! this would not compile 
                    //   because it does not return a u32
    }
}





// NUMBERS
use std::f32;
use std::f64;
let largest: u64 = 0xfffffffffffff;
 let zero: u64 = 0x10000000000000;
let num = 1_000_000u64;

let flt = 123.4f64;     // Double-like
let fp2 = 0.1f32;       // Float-like
let fp3 = 12E+99_f64;   //Exponents
let fp4 = 1e0;          // 1.0
let fp5 = 1.3e-1;       // 
let fp = 123e+10;
let fp = 1.0e-40_32;

let lowest = f32::MIN_POSITIVE;
let max = f32::MAX;

format!("", 1.0) == "";
// https://github.com/rust-lang/rust/issues/10843
format!("{:e}", 1.0) == "1e0";
format!("{:e}", 10.0) == "1e1";
format!("{:+e}", 10.0) == "+1e+1";
format!("{:e}", 1.1234567) == "1.123457e0";
format!("{:e}", 1.3e-1) == "1.3e-1";
format!("{:.2e}", 1.3e-1) == "1.30e-1";
format!("{:E}", 1.3e-1) == "1.3E-1";





// NUMBER STRING CONVERSIONS
let str_to_int = u64::from_str_radix(&"10", 10).expect("Not an integer");
let int_string = "1000000".to_string();
let parse_int: u64 = "1000000".parse().unwrap();
let parse_int: u64 = "1000000".parse::<u64>().unwrap(); // Turbo fish syntax ::<>()
let padded = format!("{:08.2}", 1000.1234); // 00001000.12
let pad_left = format!("{txt:=<width$}", txt="text", width=20); // left padded to a variable place with = as padding
let pad_right = format!("{:=>7}", "text"); // right padded to 7 places with = as padding
let justified = format!("{:=^width$}", "text", 12); // centered with = on both sides





// TRIM STRINGS
"  text  ".trim()
"    text".trim_left()
"text    ".trim_right()





// BYTE AND RAW LITERALS
let byte_data = b"This is a string of bytes";
let raw_string = r##"This "can" have weird \"characters" in it 'no' problem"##;
let raw_byte = br##"you can even mix and match"##;





// CONVERT BYTES VECTOR & STRINGS
let data = "abcdefghijklmnopqrstuvwxyz";
let data_ref = data.as_bytes();
let data_owned = data.into_bytes();





// ERROR HANDLING
let result = Ok("Some Text").unwrap_or("Fall-back text");
let rst = Ok("Text text text").unwrap();
let rst = Ok("Text text text").expect("Panic message");

// The ? operator means unwrap or return Err(From::from(err))
fn example(num: String) -> Result<u64> {
    let parsed = num.parse::<u64>()?; // Returns Err() if it fails to unwrap
    Ok(parsed + 1)
}





// TRAITS

// Inheritance: Traits that require implementations to implement other traits
//   require the implementations to also implement SomeTrait and ThingTrait
trait ThisTrait: SomeTrait + ThingTrait 

impl SomeTrait for Thing<u32> { /* ... */ }

// Trait bounds
fn foo<T: Clone, U: Debug + Clone + Default>(a: T, b: U) -> T { /* ... */ }
fn foo<T>(a: T, b: U) -> T where T: Clone, U: Debug + Clone + Default { /* ... */ }

// Associated Constants
trait Foo {
    const ID: u32;
    // or with a default value:
    const UID: u32 = 0;
}
impl Foo for u32 {
    const ID: u32 = 5
}




// TRAIT OBJECTS
//   &SomeTrait or Box<SomeTrait> are trait objects

// To have a trait used as a trait objcet it must:
//   not require that Self: Sized
//   all of its methods are object safe, which means:
//     must not have any type parameters
//     must not use Self
//       traits that use Self are not Object safe
trait Shape { }
impl Shape for i32 { }
let mycircle = 0i32;
let myshape: Box<Shape> = Box::new(mycircle) as Box<Shape>;

// Or
trait Printable {
    fn stringify(&self) -> String;
}
impl Printable for i32 {
    fn stringify(&self) -> String { self.to_string() }
}
fn print(a: Box<Printable>) {
    println!("{}", a.stringify());
}
fn main() {
    print(Box::new(10) as Box<Printable>);
}





// Generic Type Parameters
fn do_something<A: Clone + SomeTrait>(a: &[A], b: Vec<A>) -> A {
    // SomeTrait must define the some() method
    if A::some(a) {
        let data: A = a.thing();
    } else {
        let info: A = Vec<A>;
    }
}




// DERIVABLE TRAITS
#[derive(Clone)]
/* Derivable traits
    Clone
    Copy
    Hash
    Default
    Zero = a zero instance of a numeric type
    Debug
    
   Comparison Traits
    Eq
    Requires PartialEq. Reflexive (a==a) Symmetric (a==b implies b==a) and Transative (a==b & b==c so a==c)
    Has no methods!!
    
    PartialEq   
    Symmetric (a==b implies b==a) and Transative (a==b & b==c so a==c) but NOT REFLEXIVE (a==a)
    requires the eq method
    ne (not equal) is defined as the inverse of this
    
    Ord    
    Must also be PartialOrd and Eq (Eq requires PartialEq).  Total order, must be exactly one of a<b, a==b, a>b and transitive: a<b b<c so a<c
    requires the PartialOrd and Eq traits, then implement cmp
    
    PartialOrd  
    Antisymmetry (if a<b then !(a>b) as well as a>b then !(a<b))  and transivity (a<b & b<c so a<c)
    requires the partial_cmp method be implemented
*/
impl PartialEq for Book {
    fn eq(&self, other: &Book) -> bool {
        self.id == other.id
    }
}
use std::cmp::Ordering;
// using cmp
impl PartialOrd for Book {
    fn partial_cmp(&self, other: &Book) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
// using partial_cmp
impl PartialOrd for Book {
    fn partiall_cmp(&self, other: &Book) -> Option<Ordering> {
        self.title.partial_cmp(&other.title)
    }
}
impl Ord for Book {
    fn cmp(&self, other: &Book) -> Ordering {
        self.title.cmp(&other.title)
    }
}
// Try #[derive(Eq)] first, if you cannot use that implement PartialEq's eq() and use:
impl Eq for Book {} // Eq has no methods cuz it's reflexive meaning a==a

// FORMAT DISPLAY - fmt::Display
impl std::fmt::Display for MonsterType {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "Something");
    }
}




// OVERLOADABLE TRAITS
// Add(+), AddAssign(+=), BitAnd(&), BitAndAssign(&=), BitOr(|), BitOrAssign(|=), BitXor(^), BitXorAssign(^=)
use std::ops::Add;
impl Add for Point {
    type Output = Point; // The resulting type after applying the add operating
    fn add(self, other: Point) -> Point {
        Point {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}
// Generic Add
struct Point<T> { x: T, y: T, }
impl<T: Add<Output=T>> Add for Point<T> {
    type Output = Point<T>; // The resulting type after applying the add operating
    fn add(self, other: Point<T>) -> Point<T> {
        Point {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}
// Xor: only one of the values is true.  true^true = false, false^false=false, true^false=true, false^true=true
use std::ops::BitXorAssign;
impl BitXorAssign for Color {
    fn bitxor_assign(&mut self, rhs: Self) {
        // bitxor the rhs to the lhs
    }
}
// https://doc.rust-lang.org/std/ops/trait.BitXor.html
#[derive(Debug, PartialEq)]
struct BoolVec(Vec<bool>);
impl BitXor for BoolVec {
    // The method takes self for the left hand side, the right side is:
    //     a BoolVec, however only the insides matter, and 
    fn bitxor(self, BoolVec(rhs): Self) -> Self {
        
    }
}





// PROGRAM ARGUMENTS 
use std::env;
let args: Vec<String> = env::args().collect();





// RANDOM NUMBERS
extern crate rand;
use rand::{thread_rng, Rng};
use rand::distributions::range::SampleRange;
use num::{Num, Zero, One};
use std::ops::Add;

// Use the num crate to add one to the generic number
pub fn safe_range<N>(starting: N, ending: N) -> N where N: Num 6 PartialOrd + Copy + SampleRange {
    let end_plus = ending + N::one();
    if start < end {
        let mut rg = thread_rng();
        // gen_range is [start, end) meaning it returns numbers
        //      as low as start and up to one less than end
        rg.gen_range(starting, end_plus); 
    }
}

// Most basic random number using modulus operator
let rand_num: u8 = (rand::random() % 100) + 1 // random number 1-100
let rand_num = (rand::random::<u8>() % 100) + 1 // without +1 it would give 0-99

// Choose a random item
let mut rg = thread_rng();
let gcat = rg.choose(&items);

// Shuffling
let mut rg = thread_rng();
let mut items = vec![0, 1, 2, 3, 4, 5, 6, 7, 8, 9];
rg.shuffle(&mut items);





// LOGGING
// Run with RUST_LOG=info or RUST_LOG=traitstr=info or RUST_LOG=main=info
// Types are: trace, debug, info, warn, error
#[macro_use] extern crate log;
extern crate env_logger;

fn main() {
    env_logger::init();
}



// Todo: VERSIONING
extern crate semver;





// WINDOWS REGISTRY
extern crate winreg;
use winreg::RegKey;
use winreg::enums::*;





// TIMING
extern crate time;
use std::time::Instant;

let start = Instant::now();

let end = start.elapsed();
println!("Served in {}.{:08} seconds", end.as_secs(), end.subsec_nanos());





// REGEX
#[macro_use] extern crate lazy_static;
extern crate regex;

use regex::Regex;

lazy_static! {
    static ref RE: Regex = Regex::new(r#"^[A-Za-z0-9]$"#).unwrap();
}





// SERIALIZATION
extern crate serde;
#[macro_use] extern crate serde_derive;

use std::path::{Path, PathBuf};
use std::fs::File;
use std::io::{Write, Read};
use ::serde::{Deserialize, Serialize};
use ::rmps::{Deserializer, Serializer};
use serde_json::Error;

// replace path & data with file and data structure to serialize
let mut f = File::create(path).expect("File create failed.");

// YAML
let ser = ::serde_yaml::to_string(data).expect("Could not serialize");
let ser = ::serde_yaml::to_vec(data).expect("Could not serialize");

// JSON
let ser = ::serde_json::to_string_pretty(data).expect("Could not serialize");
let ser = ::serde_json::to_string(data).expect("Could not serialize");
let ser = ::serde_json::to_vec_pretty(data).expect("Could not serialize");
let ser = ::serde_json::to_vec(data).expect("Could not serialize");

// MSGPACK
let mut ser: Vec<u8> = Vec::new();
data.serialize(&mut Serializer::new(&mut ser_buf)).expect("Could not serialize");
let mut ser: String = String::new();

// write string to file
let rst = f.write(ser.as_bytes());
// write vector to file
let rst = f.write(&ser);
// Result
if let Ok(res) = rst { if res != 0 { true } else { false } } else { false }





// FILE WRITING
let mut f = BufWrtier::new(File::create(filename).expect("Could not create file"));

let text = include_bytes!("filename.txt");
f.write(text);

let text2 = include_str!("filename.txt");
f.write(text2.as_bytes());

let data = vec![b"this will be represented as a vector of bytes"];
f.write(&data[..]);


// BUFFERED FILE STUFF
use std::io::{BufReader, BufRead, BufWriter, Write};
use std::fs::File;
use std::prelude::*;
let mut f = BufReader::new(File::open("filename.txt").expect("Could not open file"));
for line in f.lines() {
    match file_line {
        Err(e) => println!("Error: {}", e);
        Ok(line) => {
            // ...
        }
    }
}





// GLOB FILE LISTING
// https://stackoverflow.com/questions/26076005/how-can-i-list-files-of-a-directory-in-rust
extern crate glob;
use self::glob::glob;

fn main() {
    let files:Vec<Path> = glob("*").collect();
}





// IO / CONSOLE INPUT
use std::io::{BufReader, BufRead, BufWriter, Write};
use std::fs::File;
use std::prelude::*;
use std::io::{self, stdin, Read};

let sin = io::stdin();
let mut buf = String::new();
let mut name: String = String::new();
println!("Enter something");
match sin.read_line(&mut buf) {
    Ok(_) => {
        name = buf.trim().to_string();
        buf.clear();
    },
}
//





// RUNNING EXECUTABLES
use std::process::{Command, Output, ExitStatus};

let cmd = Command::new("passgen")
    .args(&["-e", "-c", "-l 10", "-r 20"])
    .output()
    .expect("Failed to execute command");
//





// ATTRIBUTES: OPERATING SYSTEM TARGETS & CONDITIONAL COMPILATION
if cfg!(target_os = "windows") { /* ... */ }
#[cfg(target_os = "linux")] // can be linux android windows macos ios
#[cfg(target_pointer_width = 64)] // target 64 bit systems
#[cfg(target_pointer_width = 32)] // target 32 bit systems

// Compiler features
#![feature(feature1, feature2, feature3)]
#[cfg(feature = "foo")]

// combining multiple conditions
#[cfg(any(unix, windows))]
#[cfg(not(macos))]
#[cfg(not(unix), all(target_os="macos", target_arch = "powerpc"))]

#![plugin(foo, bar)]

// Load a module from a specified file
#[path = "foo.rs"]
mod bar;

// Set crate type to library
#![crate_type = "lib"]

// You can set attributes based on a cfg variable
#[cfg_attr("is_cool", windows)] // sets the is_cool attribute if compiling for windows


/* https://doc.rust-lang.org/reference/attributes.html#static-only-attributes
   Crate Attributes
    crate-name
    crate-type
    feature
    no_builtins - for not using stdlib
    no_main - disable emitting main symbol, use when another object being linked to defines main
    no_start - disable linking to the native crate which specifies the start language item
    no_std - disable linking to the std crate
    plugin - load a list of named crates as compiler plugins
    recursion_limit - set max depth for recursive compile-time operations like auto-dereference or macro expansion
    windows_subsystem - either console or windows - indicates when this crate is linked for a windows target it will configure the binary's subsystem
        Subystem on MSDN:   https://msdn.microsoft.com/en-us/library/fcc1zstk.aspx

   Module Attributes
    no_implicit_prelude - disable inject use std::prelude::* in this module
    path - specifies the file to load the module from

   Function Attributes
    main - indicates that the function should be passed to the entry point instead of the main function in the crate root
    plugin_registrar - marks this as the registration point for compiler plugins
    start - indicates that this function should be used as the entry point, override the start language item
    test - indicates a test function
        ignore - ignore this test unless used with --ignored
        should_panic - indicates a test should panic, a panic indicates test passes
    cold - the function is unlikely to be executed so optimize it (and calls to it) differently
    naked - the function utilizes a custom ABI or custom ASM that requires epilogue and prologue to be skipped

   Static Attributes
    thread_local - used on a static mut - signals that the value of this static may change depending on current thread.

   FFI Attributse
    link_args - specify arguments to the linker
    link - indicate native library should be linked to for the declarations in this block
        #[link(name = "readline")]
        #[link(name = "CoreFoundation", kind = "framework")]
            kind can be: dylib, static, and framework
    linked_from - indicates what native library this block of FFI items is coming from
        #[linked_from = "foo"]  // foo is the name of a library in either #[link] or a -l flag
    repr - specifies the underlying representation
        enums: can be the primitive type the enum should be represented with, 
            or C which specifies it should be the default enum size of the C ABI for the platform
        structs: specify `C` representation and/or `packed`
            C is a C ABI compatible struct layout
            packed will remove any padding between fields - this is fragile and may break platforms which require aligned access

   Miscellaneous Attributes
    deprecated - marks an item as deprecated
        #[deprecated(since = "1.0.0", note = "Deprecated as of version blah")]
    export_name - on statics and functions this specifies the exported symbol name
    link_section - on statics and functions this specifies the section of the object file that this item's contents will be placed into
    no_mangle - on any item this indicates that the name should not be mangled
    simd - on certain tuple structs, derive the arithmetic operators which lower to the targets SIMD instructions, if any
        the simd feature gate is necessary to use this
    unsafe_destructor_blind_to_params - on Drop::drop asserts that the destructor code will never attempt to read from nor write to any references with ilfetimes that come in via generic parameters
    doc - Doc comments such as /// foo  are equivelant to #[doc = "foo"]
    rustc_on_unimplemented - write a custom note to be shown along with the error when the trait is found to be unimplemented on a type
        can use args like {T}, {A}, {Self} to correspond to the types
        For trait Foo in Module Bar {Foo} shows up Bar::Foo
    must_use - for structs and enums will warn if a value of this type isn't used or assigned to a variable
        #[must_use = "Optional message too"]
    
   Implementation Defined Attributes
    target_arch - Can be: x86, x86_64, mips, powerpc, powerpc64, arm, aarch64
    target_os - Can be: windows, macos, ios, linux, android, freebsd, dragonfly, bitrig, openbsd, netbsd
    target_family - Can be: unix, windows
    target_env - Tells which ABI/libc is used.  Can be: gnu, msvc, musl or ""
    target_endian - Endianness of the target CPU.  Can be either little or big
    target_pointer_width - Can be 32 or 64 depending on the system architecture
    target_has_atomic - Set of integer sizes on which the target can perform atomic operations.  Can be: 8, 16, 32, 64, and ptr
    debug_assertions - Enabled by default when compiling without opimizations.
        This can be used to enable extra debugging code in development but not production.
        This can control the behavior of the std library debug_assert! macro
    
*/





// DESERIALIZATION
extern crate serde;
#[macro_use] extern crate serde_derive;
extern crate serde_json;
extern crate rmp_serde as rmps;
extern crate serde_yaml;

use std::path::{Path, PathBuf};
use std::fs::File;
use std::io::{Write, Read};
use ::serde::{Deserialize, Serialize};
use ::rmps::{Deserializer, Serializer};
use serde_json::Error;

let mut open = File::open(path).expect("Could not open file");

// Read as string
let mut des_buf: String = String::new();
f.read_to_string(&mut des_buf);
// Read as vector
let mut des_buf: Vec<u8> = Vec::new();
f.read_to_end(&mut des_buf);

// YAML
let des: Test = ::serde_yaml::from_str(&des_buf).expect("Could not deserialize");
let des: Test = ::serde_yaml::from_slice(&des_buf).expect("Could not deserialize");

// JSON
let des: Test = ::serde_json::from_slice(&mut des_buf).expect("Could not deserialize");
let des: Test = ::serde_json::from_str(&mut des_buf).expect("Could not deserialize");

// MSGPACK
// for sure works:
let mut de = Deserializer::new(&des_buf[..]);
let des: Test = Deserialize::deserialize(&mut de).expect("Could not deserialize");
// not tested:
let des: Test = Deserialize::deserialize(&mut Deserialize::deserialize(&des_buf[..])).expect("Could not deserialize");
// might work    ::rmps::decode::from_slice(Deserialize::dserialize::new(&des_buf[..])).expect("Could not deserialize");





// ARGUMENT PARSING
extern crate argparse;
use argparse::{ArgumentParser, StoreFalse, StoreTrue, Store};
let mut vart = false;
let mut varf = true;
let mut vars = String::from("");
{
    let mut ap = ArgumentParser::new();
    ap.set_description("Program description");
    ap.refer(&mut vart).add_option(&["-t", "--true"], StoreTrue, "description");
    ap.refer(&mut varf).add_option(&["-f", "--false"], StoreFalse, "description");
    ap.refer(&mut vars).add_option(&["-s", "--store"], Store, "description");
    
    ap.parse_args_or_exit();
}




// FUNCTION POINTERS
fn call_fn<F>(func: F, data: usize) -> bool where F: Fn(usize) -> bool {
    func(data)
}
let is_even = |n| n%2==0;
assert!(call_fn(is_even, 10));






// SCAN_DIR CRATE - FILE HANDLING
// http://tailhook.github.io/scan_dir/scan_dir/index.html
extern crate scan_dir;
use scan_dir::ScanDir;

// possibly completely wrong, may inspect and rewrite later. maybe.  if i feel amitious.
ScanDir::files().read(path.to_str().unwrap_or(".")), |mut iter| {
    let items: Vec<String> = iter.by_ref()
        .map(|x| x.1)
        .filter(|ref n| ext_filter(na, ctx))
        .collect();
}
// get a list of not hidden rust files
let files: Vec<_> = ScanDir::files().read("." |iter| {
    iter.filter(|&(_, ref name)| name.ends_with(".rs"))
        .map(|(entry, _) entry.path())
        .collect()
}).unwrap();





// GLOBAL MUTABLE VARiABLE
// this is bad.  do not do this.
use std::mem;

// Put these in main.rs/lib.rs inside the main() to ensure they exist for entire program
static mut MY_GLOBAL: *const Vec<String> = 0 as *const Vec<String>;
fn main() {
    let mut initialized_my_global: Vec<String> = Vec::new();
    unsafe {
        MY_GLOBAL = mem::transmute(&initialized_my_global);
    }
}

// Elsewhere in the program
let mut my_global: &mut Vec<String>;
unsafe {
    my_global = mem::transmute(MY_GLOBAL);
}





// External Blocks
// https://doc.rust-lang.org/reference/items.html#external-blocks
/* Externs
    Cross platform:
        extern "Rust" - Default ABI 
        extern "C" - C ABI - Same as extern fn foo();
        extern "system" - Usually same as extern "C" except on win32 where it's stdcall or what you should use to link to the Windows API
    Platform specific:
        extern "cdecl" - default for x86_32 code
        extern "stdcall" - default for Win32 API on x86_32
        extern "win64" - default for C code on x86_64 Windows
        extern "sysv64" - default for C code on non-windows x86_64
        extern "aapcs" - default for ARM
        extern "fastcall" - the fastcall ABI - corresponds to MSVS's __fastcall and GCC and Clang's __attribute__((fastcall)) 
        extern "vectorcall" - The vectorcall ABI - corresponds to MSVC's __vectorcall and Clang's __attribute__((vectorcall))
    Rustc Specific ABI
        extern "rust-intrinsic" - The ABI of rustc intrinsics
        extern "rust-call" - The ABI of the Fn::call trait functions
        extern "platform-intrinsic" - Specific platform intrinsics (like sqrt) have this ABI.  
            You should not have to deal with this.
            
*/
// Specify the name of the native library.
#[link(name = "crypto")]
extern { /* ... */ }
extern "abi" fn(A1, ..., An) -> R // specifies 















