//! Synchronized state abstraction traits for platform-agnostic state access.
//!
//! This module provides the `SharedState` trait that abstracts over different
//! synchronization mechanisms (Embassy Mutex, RefCell for tests) to enable
//! host testing without embedded dependencies.

/// Platform-agnostic synchronized state access.
///
/// This trait abstracts over different synchronization mechanisms:
/// - `EmbassyState<T>` for embedded targets using Embassy's critical-section Mutex
/// - `MockState<T>` for host testing using RefCell (single-threaded)
///
/// # Example
///
/// ```ignore
/// struct Counter {
///     value: u32,
/// }
///
/// fn increment<S: SharedState<Counter>>(state: &S) -> u32 {
///     state.with_mut(|counter| {
///         counter.value += 1;
///         counter.value
///     })
/// }
/// ```
pub trait SharedState<T> {
    /// Access state immutably.
    ///
    /// The provided closure receives an immutable reference to the inner state.
    fn with<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&T) -> R;

    /// Access state mutably.
    ///
    /// The provided closure receives a mutable reference to the inner state.
    fn with_mut<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut T) -> R;
}

// ============================================================================
// Embassy Implementation
// ============================================================================

#[cfg(feature = "embassy")]
use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};

/// Embassy-based synchronized state using critical-section Mutex.
///
/// This implementation uses Embassy's blocking mutex with critical sections
/// for interrupt-safe state access on embedded targets.
///
/// # Safety
///
/// The critical section ensures atomic access even in interrupt contexts,
/// making this safe for use in async tasks and interrupt handlers.
#[cfg(feature = "embassy")]
pub struct EmbassyState<T> {
    inner: Mutex<CriticalSectionRawMutex, core::cell::RefCell<T>>,
}

#[cfg(feature = "embassy")]
impl<T> EmbassyState<T> {
    /// Creates a new `EmbassyState` wrapping the given value.
    ///
    /// This is a const fn, allowing static initialization.
    pub const fn new(value: T) -> Self {
        Self {
            inner: Mutex::new(core::cell::RefCell::new(value)),
        }
    }
}

#[cfg(feature = "embassy")]
impl<T> SharedState<T> for EmbassyState<T> {
    fn with<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&T) -> R,
    {
        self.inner.lock(|cell| f(&cell.borrow()))
    }

    fn with_mut<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut T) -> R,
    {
        self.inner.lock(|cell| f(&mut cell.borrow_mut()))
    }
}

// ============================================================================
// Mock Implementation (always available for testing)
// ============================================================================

/// Mock synchronized state using RefCell for single-threaded testing.
///
/// This implementation uses `RefCell` for runtime borrow checking,
/// suitable for single-threaded test environments.
///
/// # Panics
///
/// Panics if borrowing rules are violated (e.g., calling `with_mut` while
/// `with` is active). This indicates a bug in the test code.
///
/// # Example
///
/// ```
/// use pico_trail::core::traits::sync::{MockState, SharedState};
///
/// let state = MockState::new(42u32);
///
/// // Immutable access
/// let value = state.with(|v| *v);
/// assert_eq!(value, 42);
///
/// // Mutable access
/// state.with_mut(|v| *v += 1);
/// assert_eq!(state.with(|v| *v), 43);
/// ```
pub struct MockState<T> {
    inner: core::cell::RefCell<T>,
}

impl<T> MockState<T> {
    /// Creates a new `MockState` wrapping the given value.
    pub fn new(value: T) -> Self {
        Self {
            inner: core::cell::RefCell::new(value),
        }
    }
}

impl<T> SharedState<T> for MockState<T> {
    fn with<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&T) -> R,
    {
        f(&self.inner.borrow())
    }

    fn with_mut<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut T) -> R,
    {
        f(&mut self.inner.borrow_mut())
    }
}

// ============================================================================
// Unit Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mock_state_with_read() {
        let state = MockState::new(42u32);
        let value = state.with(|v| *v);
        assert_eq!(value, 42);
    }

    #[test]
    fn mock_state_with_mut_write() {
        let state = MockState::new(0u32);
        state.with_mut(|v| *v = 100);
        assert_eq!(state.with(|v| *v), 100);
    }

    #[test]
    fn mock_state_with_struct() {
        #[derive(Default)]
        struct TestState {
            counter: u32,
            name: &'static str,
        }

        let state = MockState::new(TestState {
            counter: 0,
            name: "test",
        });

        // Read fields
        assert_eq!(state.with(|s| s.counter), 0);
        assert_eq!(state.with(|s| s.name), "test");

        // Modify fields
        state.with_mut(|s| {
            s.counter = 5;
            s.name = "modified";
        });

        assert_eq!(state.with(|s| s.counter), 5);
        assert_eq!(state.with(|s| s.name), "modified");
    }

    #[test]
    fn mock_state_multiple_reads() {
        let state = MockState::new(42u32);

        // Multiple sequential reads should work fine
        let v1 = state.with(|v| *v);
        let v2 = state.with(|v| *v);
        assert_eq!(v1, v2);
    }

    #[test]
    fn mock_state_read_modify_read() {
        let state = MockState::new(0u32);

        // Read
        assert_eq!(state.with(|v| *v), 0);

        // Modify
        state.with_mut(|v| *v += 10);

        // Read again
        assert_eq!(state.with(|v| *v), 10);
    }

    #[test]
    fn mock_state_closure_return_value() {
        let state = MockState::new(vec![1, 2, 3]);

        // Return computed value from closure
        let sum: i32 = state.with(|v| v.iter().sum());
        assert_eq!(sum, 6);

        // Return modified state info
        let new_len = state.with_mut(|v| {
            v.push(4);
            v.len()
        });
        assert_eq!(new_len, 4);
    }

    #[test]
    #[should_panic(expected = "already borrowed")]
    fn mock_state_double_borrow_panics() {
        let state = MockState::new(0u32);

        // This should panic due to borrow conflict
        state.with(|_v| {
            // Try to get mutable borrow while immutable borrow is held
            // Using inner directly to test RefCell behavior
            let _ = state.inner.borrow_mut();
        });
    }
}
