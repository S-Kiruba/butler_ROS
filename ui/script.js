document.addEventListener('DOMContentLoaded', () => {
    // Elements
    const loginSection = document.getElementById('login_section');
    const dashboardSection = document.getElementById('dashboard_section');
    const vendorDashboard = document.getElementById('vendor_dashboard');
    const customerDashboard = document.getElementById('customer_dashboard');

    const loginButton = document.getElementById('login_button'); // Single login button
    const logoutButton = document.getElementById('logout_button'); // Single logout button

    const tableSelectVendor = document.getElementById('table_select_vendor');
    const tableSelectCustomer = document.getElementById('table_select_customer');

    let currentUserType = ''; // Tracks if the user is vendor or customer
    const vendorCredentials = { username: "vendor", password: "vendor123" };
    const customerCredentials = { username: "customer", password: "customer123" };

    let totalTables = 4; // Initial table count
    let tableConfirmations = {}; // Confirmation data for tables
    let taskQueue = []; // Task queue for orders

    // Function to dynamically generate table options for dropdowns
    function generateTableOptions() {
        const vendorSelect = document.getElementById('table_select_vendor');
        const customerSelect = document.getElementById('table_select_customer');

        // Clear existing options
        vendorSelect.innerHTML = '';
        customerSelect.innerHTML = '';

        for (let i = 1; i <= totalTables; i++) {
            const vendorOption = document.createElement('option');
            vendorOption.value = i;
            vendorOption.textContent = `Table ${i}`; // No space between "Table" and number
            vendorSelect.appendChild(vendorOption);

            const customerOption = document.createElement('option');
            customerOption.value = i;
            customerOption.textContent = `Table ${i}`; // No space between "Table" and number
            customerSelect.appendChild(customerOption);
        }
    }

    // Login functionality (single button for both vendor and customer)
    loginButton.addEventListener('click', () => {
        const username = document.getElementById('username').value;
        const password = document.getElementById('password').value;

        if (username === vendorCredentials.username && password === vendorCredentials.password) {
            currentUserType = 'vendor';
            loginSection.style.display = 'none';
            dashboardSection.style.display = 'block';
            vendorDashboard.style.display = 'block';
            generateTableOptions();
        } else if (username === customerCredentials.username && password === customerCredentials.password) {
            currentUserType = 'customer';
            loginSection.style.display = 'none';
            dashboardSection.style.display = 'block';
            customerDashboard.style.display = 'block';
            generateTableOptions();
        } else {
            alert("Invalid Username or Password.");
        }
    });

    // Logout functionality (single button for both vendor and customer)
    logoutButton.addEventListener('click', () => {
        loginSection.style.display = 'block';
        dashboardSection.style.display = 'none';
        vendorDashboard.style.display = 'none';
        customerDashboard.style.display = 'none';
        document.getElementById('username').value = '';
        document.getElementById('password').value = '';
        currentUserType = '';
    });

    // Vendor: Place order
    document.getElementById('place_order_button').addEventListener('click', () => {
        const tableNumber = tableSelectVendor.value;
        if (tableNumber) {
            const order = `Table${tableNumber}`; // Consistent format
            taskQueue.push(order);
            updateTaskQueue();

            // Send order to backend
            fetch('http://localhost:5000/order', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ order })
            })
            .then(response => response.json())
            .then(data => console.log(data))
            .catch(error => console.error('Error:', error));

            alert(`Order placed for Table${tableNumber}.`);
        } else {
            alert("Please select a table to place an order.");
        }
    });

    // Vendor: Cancel order
    document.getElementById('cancel_order_button').addEventListener('click', () => {
        const tableNumber = tableSelectVendor.value;
        if (tableNumber) {
            const orderToCancel = `Table${tableNumber}`; // Consistent format

            const index = taskQueue.indexOf(orderToCancel);
            if (index !== -1) {
                taskQueue.splice(index, 1);
                updateTaskQueue();

                // Send cancel request to backend
                fetch('http://localhost:5000/cancel', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({ task: orderToCancel })
                })
                .then(response => response.json())
                .then(data => console.log(data))
                .catch(error => console.error('Error:', error));

                alert(`Order cancelled for Table${tableNumber}.`);
            } else {
                alert("No order found for the selected table.");
            }
        } else {
            alert("Please select a table to cancel the order.");
        }
    });

    // Vendor: Confirm table readiness (Push button configuration)
    document.getElementById('confirm_table_vendor_button').addEventListener('click', () => {
        const tableNumber = tableSelectVendor.value;
        if (tableNumber) {
            tableConfirmations[tableNumber] = true; // Set confirmation to true
            alert(`Table ${tableNumber} confirmed by vendor.`);

            // Send confirmation to backend
            fetch('http://localhost:5000/table_confirm', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(true),
            })
            .then(response => response.json())
            .then(data => console.log(data))
            .catch(error => console.error('Error:', error));

            // Reset confirmation to false after 2 seconds
            setTimeout(() => {
                tableConfirmations[tableNumber] = false;
                updateTableConfirmationDisplay();
            }, 2000);

            updateTableConfirmationDisplay();
        } else {
            alert("Please select a table to confirm.");
        }
    });

    // Vendor: Confirm kitchen readiness (Push button configuration)
    document.getElementById('confirm_kitchen_button').addEventListener('click', () => {
        const tableNumber = tableSelectVendor.value;
        if (tableNumber) {
            alert(`Kitchen readiness confirmed for Table ${tableNumber}.`);

            // Send kitchen confirmation to backend
            fetch('http://localhost:5000/kitchen_confirm', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(true),
            })
            .then(response => response.json())
            .then(data => console.log(data))
            .catch(error => console.error('Error:', error));
        } else {
            alert("Please select a table to confirm.");
        }
    });

    // Customer: Confirm table seating (Push button configuration)
    document.getElementById('confirm_table_customer_button').addEventListener('click', () => {
        const tableNumber = tableSelectCustomer.value;
        if (tableNumber) {
            tableConfirmations[tableNumber] = true; // Set confirmation to true
            alert(`Table ${tableNumber} confirmed by customer.`);

            // Send confirmation to backend
            fetch('http://localhost:5000/table_confirm', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(true),
            })
            .then(response => response.json())
            .then(data => console.log(data))
            .catch(error => console.error('Error:', error));

            // Reset confirmation to false after 2 seconds
            setTimeout(() => {
                tableConfirmations[tableNumber] = false;
                updateTableConfirmationDisplay();
            }, 2000);

            updateTableConfirmationDisplay();
        } else {
            alert("Please select a table to confirm.");
        }
    });

    // Vendor: Add new table
    document.getElementById('add_table_button').addEventListener('click', () => {
        totalTables += 1;
        generateTableOptions();
        alert(`Table ${totalTables} added.`);
    });

    // Vendor: Remove a table
    document.getElementById('remove_table_button').addEventListener('click', () => {
        const tableNumber = tableSelectVendor.value;
        if (tableNumber && totalTables > 1) {
            totalTables -= 1;
            generateTableOptions();
            alert(`Table ${tableNumber} removed.`);
        } else {
            alert("Cannot remove the last table or no table selected.");
        }
    });

    // Helper function to update the task queue display
    function updateTaskQueue() {
        const taskQueueDiv = document.getElementById('task_queue');
        taskQueueDiv.innerHTML = '';
        taskQueue.forEach(task => {
            const taskItem = document.createElement('div');
            taskItem.textContent = task;
            taskQueueDiv.appendChild(taskItem);
        });
    }

    // Helper function to update the table confirmation display
    function updateTableConfirmationDisplay() {
        const vendorConfirmationDiv = document.getElementById('vendor_table_confirmations');
        const customerConfirmationDiv = document.getElementById('table_confirmation_customer');

        // Update vendor confirmations
        vendorConfirmationDiv.innerHTML = '';
        for (let table in tableConfirmations) {
            const confirmationItem = document.createElement('div');
            confirmationItem.textContent = `Table ${table}: ${tableConfirmations[table] ? 'Confirmed' : 'Not Confirmed'}`;
            vendorConfirmationDiv.appendChild(confirmationItem);
        }

        // Update customer confirmations
        customerConfirmationDiv.innerHTML = '';
        for (let table in tableConfirmations) {
            const confirmationItem = document.createElement('div');
            confirmationItem.textContent = `Table ${table}: ${tableConfirmations[table] ? 'Confirmed' : 'Not Confirmed'}`;
            customerConfirmationDiv.appendChild(confirmationItem);
        }
    }
});

