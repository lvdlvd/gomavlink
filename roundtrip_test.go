package gomavlink_test

import (
	"bytes"
	"math"
	"math/rand/v2"
	"reflect"
	"strings"
	"testing"

	mavlink "github.com/lvdlvd/gomavlink"
	all "github.com/lvdlvd/gomavlink/all"
)

// maxMsgID is one past the highest message id present in the 'all' dialect.
const maxMsgID = 60054

// wireBits returns the number of bits the field occupies on the wire, derived
// from the generated `mavlink:"..."` struct tag. The tag value is the C wire
// type, optionally prefixed with an array dimension and/or suffixed with
// ",bitmask"; e.g. "byte", "uint32,bitmask", "[5]uint16". It returns 0 when the
// field carries no tag, meaning the Go type already matches the wire width.
func wireBits(tag string) int {
	s, ok := reflect.StructTag(tag).Lookup("mavlink")
	if !ok {
		return 0
	}
	if i := strings.IndexByte(s, ','); i >= 0 { // drop ",bitmask"
		s = s[:i]
	}
	if i := strings.IndexByte(s, ']'); i >= 0 { // drop "[N]" array prefix
		s = s[i+1:]
	}
	switch s {
	case "byte", "char", "int8", "uint8":
		return 8
	case "int16", "uint16":
		return 16
	case "int32", "uint32":
		return 32
	case "int64", "uint64":
		return 64
	default:
		panic("wireBits: unknown wire type " + s)
	}
}

// randFill recursively fills v with deterministic pseudo random data. The Go
// types of enum/bitmask fields are often wider than the width they occupy on
// the wire, so for tagged fields we keep the value non-negative and within
// wire-width-1 bits: that round trips identically regardless of signed vs
// unsigned wire/Go types. Untagged fields (Go type == wire type) get the full
// range; float fields get fully random bit patterns so NaN/Inf are exercised.
func randFill(rng *rand.Rand, v reflect.Value, bits int) {
	switch v.Kind() {
	case reflect.Bool:
		v.SetBool(rng.Uint32()&1 == 1)
	case reflect.Int, reflect.Int8, reflect.Int16, reflect.Int32, reflect.Int64:
		if bits == 0 {
			v.SetInt(int64(rng.Uint64()))
		} else {
			v.SetInt(int64(rng.Uint64() & (uint64(1)<<(bits-1) - 1)))
		}
	case reflect.Uint, reflect.Uint8, reflect.Uint16, reflect.Uint32, reflect.Uint64:
		if bits == 0 {
			v.SetUint(rng.Uint64())
		} else {
			v.SetUint(rng.Uint64() & (uint64(1)<<(bits-1) - 1))
		}
	case reflect.Float32:
		v.SetFloat(float64(math.Float32frombits(rng.Uint32())))
	case reflect.Float64:
		v.SetFloat(math.Float64frombits(rng.Uint64()))
	case reflect.Array:
		for i := 0; i < v.Len(); i++ {
			randFill(rng, v.Index(i), bits)
		}
	case reflect.Struct:
		for i := 0; i < v.NumField(); i++ {
			randFill(rng, v.Field(i), wireBits(string(v.Type().Field(i).Tag)))
		}
	default:
		panic("randFill: unhandled kind " + v.Kind().String())
	}
}

// deepEqual is reflect.DeepEqual but treats two NaNs in a float field as equal,
// since marshalling round trips the bit pattern but NaN != NaN under ==.
func deepEqual(a, b reflect.Value) bool {
	switch a.Kind() {
	case reflect.Float32, reflect.Float64:
		x, y := a.Float(), b.Float()
		if math.IsNaN(x) && math.IsNaN(y) {
			return true
		}
		return x == y
	case reflect.Array:
		for i := 0; i < a.Len(); i++ {
			if !deepEqual(a.Index(i), b.Index(i)) {
				return false
			}
		}
		return true
	case reflect.Struct:
		for i := 0; i < a.NumField(); i++ {
			if !deepEqual(a.Field(i), b.Field(i)) {
				return false
			}
		}
		return true
	default:
		return a.Interface() == b.Interface()
	}
}

// TestRoundTrip instantiates every message in the 'all' dialect, fills it with
// random data, then checks that:
//
//  1. encode -> decode reproduces the original struct (NaN aware), and
//  2. encoding the decoded struct again yields byte identical wire output.
//
// The wire comparison accounts for the V2 encoder truncating trailing zero
// bytes from the payload: the decoder zero-pads the payload back out before
// unmarshalling, so both encode passes truncate identically and the frames
// must match exactly.
func TestRoundTrip(t *testing.T) {
	for mid := 0; mid < maxMsgID; mid++ {
		msg := all.Dialect(mid)
		if msg == nil {
			continue
		}

		rng := rand.New(rand.NewPCG(uint64(mid), 0x6d61766c696e6b))
		randFill(rng, reflect.ValueOf(msg).Elem(), 0)

		// Round trip 1: encode the random message and decode it back.
		var w1 bytes.Buffer
		enc := mavlink.NewEncoder(&w1, mavlink.Stream(42, 7, 0))
		if err := enc.Encode(msg); err != nil {
			t.Errorf("id %d (%T): encode: %v", mid, msg, err)
			continue
		}

		wire1 := append([]byte(nil), w1.Bytes()...)
		dec := mavlink.NewDecoder(bytes.NewReader(wire1), all.Dialect)
		got, _, err := dec.Decode()
		if err != nil {
			t.Errorf("id %d (%T): decode: %v", mid, msg, err)
			continue
		}

		if reflect.TypeOf(got) != reflect.TypeOf(msg) {
			t.Errorf("id %d: decoded type %T, want %T", mid, got, msg)
			continue
		}

		// Struct equality, NaN aware.
		if !deepEqual(reflect.ValueOf(msg).Elem(), reflect.ValueOf(got).Elem()) {
			t.Errorf("id %d (%T): struct mismatch after round trip\n have %+v\n want %+v",
				mid, msg, reflect.ValueOf(got).Elem(), reflect.ValueOf(msg).Elem())
			continue
		}

		// Round trip 2: re-encode the decoded struct, wire format must be identical.
		var w2 bytes.Buffer
		enc2 := mavlink.NewEncoder(&w2, mavlink.Stream(42, 7, 0))
		if err := enc2.Encode(got); err != nil {
			t.Errorf("id %d (%T): re-encode: %v", mid, msg, err)
			continue
		}

		if !bytes.Equal(wire1, w2.Bytes()) {
			t.Errorf("id %d (%T): wire mismatch after second round trip\n have %v\n want %v",
				mid, msg, w2.Bytes(), wire1)
		}
	}
}
